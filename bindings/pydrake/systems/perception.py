import numpy as np

from pydrake.math import RigidTransform
from pydrake.perception import BaseField, Fields, PointCloud
from pydrake.systems.framework import AbstractValue, LeafSystem


def _tile_colors(color, count):
    # Need manual broadcasting.
    color = np.asarray(color)
    assert color.shape == (3,)
    return np.tile(np.array([color]).T, (1, count))


def _hstack_none(A, B):
    # Concatenate, accommodating None.
    if A is None:
        return B
    else:
        return np.hstack((A, B))


class PointCloudConcatenation(LeafSystem):

    def __init__(self, id_list, default_rgb=[255., 255., 255.]):
        """
        A system that takes in a set of point clouds {point_cloudₒ_Cₒ,
        point_cloud₁_B₁, ...} measured and expressed in frames {Cₒ, C₁, ...},
        respectively, and a set of transforms {X_FCₒ, X_FC₁, ...}, where F is a
        single common frame, and outputs point_cloud_F which is the aggregation
        of {point_cloudₒ_F, point_cloud₁_F, ...}. The output point cloud will
        be guaranteed to have per-point RGB colors: each output point's color
        will either be the corresponding input point's color or the default
        color (if none is provided).

        @param id_list A list containing the string IDs of all of the point
            clouds. This is often the serial number of the camera they came
            from, such as "1" for a simulated camera or "805212060373" for a
            real camera.
        @param default_rgb A list of length 3 containing the RGB values to use
            in the absence of PointCloud.rgbs. Values should be between 0 and
            255. The default is white.

        @system{
          @input_port{point_cloud_{id₀}}
          @input_port{rigid_transform_{id₀}}
          .
          .
          .
          @input_port{point_cloud_{idₙ}}
          @input_port{rigid_transform_{idₙ}}
          @output_port{point_cloud}
        }

        Note that {idᵢ} is replaced with the string value of id_list[i].
        """
        LeafSystem.__init__(self)
        self._point_cloud_ports = {}
        self._transform_ports = {}
        self._id_list = id_list
        self._default_rgb = np.array(default_rgb)
        output_fields = Fields(BaseField.kXYZs | BaseField.kRGBs)
        for i in self._id_list:
            self._point_cloud_ports[i] = self.DeclareAbstractInputPort(
                f"point_cloud_{i}",
                AbstractValue.Make(PointCloud(fields=output_fields)))

            self._transform_ports[i] = self.DeclareAbstractInputPort(
                f"rigid_transform_{i}",
                AbstractValue.Make(RigidTransform.Identity()))
        self.DeclareAbstractOutputPort("point_cloud",
                                       lambda: AbstractValue.Make(
                                           PointCloud(fields=output_fields)),
                                       self.DoCalcOutput)

    def _align_point_clouds(self, context):
        # These will be aggregated 2D arrays.
        plist_F = None
        colors = None

        for i in self._id_list:
            point_cloudᵢ_Cᵢ = self._point_cloud_ports[i].Eval(context)
            X_FCᵢ = self._transform_ports[i].Eval(context)
            plistᵢ_Cᵢ = point_cloudᵢ_Cᵢ.xyzs()
            plistᵢ_F = X_FCᵢ.multiply(plistᵢ_Cᵢ)

            if point_cloudᵢ_Cᵢ.has_rgbs():
                colorsᵢ = point_cloudᵢ_Cᵢ.rgbs()
            else:
                colorsᵢ = _tile_colors(
                    self._default_rgb, point_cloudᵢ_Cᵢ.size())

            # Aggregate.
            plist_F = _hstack_none(plist_F, plistᵢ_F)
            colors = _hstack_none(colors, colorsᵢ)

        # Trim NaN values.
        valid_indices = np.logical_not(np.isnan(plist_F).any(axis=0))
        plist_F = plist_F[:, valid_indices]
        colors = colors[:, valid_indices]
        return plist_F, colors

    def DoCalcOutput(self, context, output):
        plist_F, colors = self._align_point_clouds(context)
        # Put points and colors into final point cloud.
        point_cloud_F = output.get_mutable_value()
        point_cloud_F.resize(plist_F.shape[1])
        point_cloud_F.mutable_xyzs()[:] = plist_F
        point_cloud_F.mutable_rgbs()[:] = colors
