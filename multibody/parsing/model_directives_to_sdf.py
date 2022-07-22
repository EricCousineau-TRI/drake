# Copyright 2022 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import sys
import warnings
import argparse
import lxml.etree as ET
from typing import Any, Dict, List

from pydrake.common.yaml import yaml_load_file

SDF_VERSION = '1.9'
SCOPE_DELIMITER = '::'
WORLD_FRAME = 'world'


class ModelDirectivesToSdf:

    # TODO(aaronchongth): Check for simple validity of output model XML,
    # whether the model just has a single joint or something, without any
    # other mandatory model elements.

    def __init__(self):
        # model_elem_map has the scoped model name as the key and its
        # corresponding XML element root as the value. This is used in
        # add_directives where model namespaces is defined, in add_model,
        # as well as when model elements are constructed due to implicit
        # or explicit frame names.
        self.model_elem_map = {}

        # incomplete_models is a list that contains all model names of model
        # elements that have been constructed but not yet valid (completed).
        # This is for keeping track that all model elements are properly
        # populated and not left empty. They are constructed using
        # add_model_instance or add_frame with scoped names.
        self.incomplete_models = []

        # child_frame_model_name_to_weld_map maps the top model instance name
        # of the child frame (in each weld) to the weld directive itself.
        # This map is used for posturing frames as the placement_frame and
        # pose combination is needed.
        self.child_frame_model_name_to_weld_map = {}

        # incomplete_models contains all directives found in the main yaml file
        self.all_directives = []

    def find_frame(self, name: str) -> str:
        """Finds and returns a frame if it exists in all directives,
           if 0 or more than one are found it returns None."""
        frame = None
        for directive in self.all_directives:
            if 'add_frame' in directive:
                add_frame_name = directive[
                    'add_frame']['name'].split(SCOPE_DELIMITER)[-1]
                if name == add_frame_name:
                    if frame is None:
                        frame = directive['add_frame']
                    else:
                        print('Found more than two frames with name:'
                              f' [{name}].')
                        return None
        return frame

    def resolve_and_scope_frame(self, frame_name: str) -> str:
        """Returns the frame scoped, None if the scope could not be found."""
        frame = self.find_frame(frame_name)
        # Check if it's atached to other frame
        final_frame = frame
        while frame is not None:
            final_frame = frame
            frame = self.find_frame(frame['X_PF']['base_frame'])

        if final_frame is not None:
            base_frame = final_frame['X_PF']['base_frame']
        else:
            return None

        split_scoped_name = base_frame.split(SCOPE_DELIMITER)[:-1]
        split_scoped_name.append(frame_name)
        return SCOPE_DELIMITER.join(split_scoped_name)

    def add_directives(self, root: ET, directive: Dict[str, Any]) -> bool:
        include_elem = None
        if 'model_namespace' in directive:
            model_ns = directive['model_namespace']
            if model_ns not in self.incomplete_models:
                print(f'Requested model namespace {model_ns} has not been '
                      'constructed.')
                return False

            if model_ns not in self.model_elem_map:
                print('The XML element of the requested model namespace '
                      f'{model_ns} has not been constructed.')
                return False

            self.incomplete_models.remove(model_ns)
            model_elem = self.model_elem_map[model_ns]
            include_elem = ET.SubElement(model_elem, 'include', merge='true')

            if model_ns in self.child_frame_model_name_to_weld_map:
                weld = self.child_frame_model_name_to_weld_map.pop(model_ns)
                placement_frame_name = \
                    weld['child'][len(model_ns)+len(SCOPE_DELIMITER):]
                include_elem.set('placement_frame', placement_frame_name)
        else:
            include_elem = ET.SubElement(root, 'include', merge='true')
        file_path = directive['file']
        result_tree = self.convert_directive(file_path)
        sdf_file_path = file_path.replace('.yaml', '.sdf')

        if result_tree is not None:
            if sdf_file_path:
                result_tree.write(
                    sdf_file_path,
                    pretty_print=True,
                    encoding="unicode")
            else:
                print(f'Error trying to guess sdf file name for'
                      'add_directives: {directive}')
                return False
        else:
            return False

        uri_elem = ET.SubElement(include_elem, 'uri')
        uri_elem.text = sdf_file_path
        return True

    def add_model(self, root: ET, directive: Dict[str, Any]) -> bool:
        model_root = root
        model_name = directive['name']
        file_name = directive['file']
        merge_include = False
        include_elem = None

        # If the model element has already been constructed due to add_frame,
        # the file is merge included.
        if model_name in self.model_elem_map:
            model_root = self.model_elem_map[model_name]
            include_elem = ET.SubElement(model_root, 'include', merge='true')
            merge_include = True
            if model_name in self.incomplete_models:
                self.incomplete_models.remove(model_name)
        else:
            include_elem = ET.SubElement(model_root, 'include')

        name_elem = ET.SubElement(include_elem, 'name')
        name_elem.text = model_name

        uri_elem = ET.SubElement(include_elem, 'uri')
        uri_elem.text = file_name

        if model_name not in self.child_frame_model_name_to_weld_map:
            return True

        # If this model contains a frame used in an add_weld instance, we
        # use the placement_frame and pose combination to posture it.
        weld = self.child_frame_model_name_to_weld_map.pop(model_name)
        scoped_parent_name_str = weld['parent']
        # Resolve frame name if implicit:
        if len(weld['parent'].split(SCOPE_DELIMITER)) == 1:
            scoped_parent_name_str = self.resolve_and_scope_frame(
                weld['parent'])
            if scoped_parent_name_str is None:
                print(f'Failed trying to find scope for frame: '
                      f'[{weld["parent"]}]. When tring to solve the '
                      f'placement_frame for model: [{model_name}].')
                return False

        if merge_include:
            model_root.set('placement_frame',
                           weld['child'].split(SCOPE_DELIMITER)[-1])
            ET.SubElement(
                model_root,
                'pose',
                relative_to=scoped_parent_name_str)
        else:
            placement_frame_elem = ET.SubElement(
                include_elem, 'placement_frame')
            placement_frame_elem.text = \
                weld['child'].split(SCOPE_DELIMITER)[-1]
            ET.SubElement(
                include_elem,
                'pose',
                relative_to=scoped_parent_name_str)
        return True

    # NOTE(aaronchongth):
    # * we can create scopes all the way down both implicit based on
    # X_PF base_frame, as well as explicit based on the scoped names,
    # how do we make sure that there is no scope violation? The base_frame
    # could be on A, however the explicit scope could be in A::B::C, the
    # frame element will then be constructed in a nested manner but
    # referencing a frame that is at the top.
    # * one solution would be comparing the explicit scoped name's scope,
    # with the scope of the base_frame, only if the base_frame lives
    # beneath the scoped_name's scope
    # (name: A::B::name, base_frame: A::B::C::frame)
    # or on the same level
    # (name: A::B::name, base_frame: A::B::frame), will it be allowed.
    # * in the implicit case, (name: name, base_frame: A::B::frame), the
    # frame will be constructed as
    # (name: A::B::name, base_frame: A::B::frame).
    def add_frame(self, root: ET, directive: Dict[str, Any]) -> bool:
        x_pf = directive['X_PF']
        base_frame = x_pf['base_frame']
        if base_frame == WORLD_FRAME:
            print(f'Workflows where base_frame="{self.WORLD_FRAME}" are not '
                  'supported to be converted using this script.')
            return False

        split_base_frame = base_frame.split(SCOPE_DELIMITER)
        # Resolve frame name if implicit:
        if len(split_base_frame) == 1:
            base_frame = self.resolve_and_scope_frame(base_frame)
            if base_frame is None:
                print(f'Failed trying to find scope for frame: '
                      f'[{x_pf["base_frame"]}]. When trying to add frame: '
                      f'[{directive["name"]}].')
                return False
            split_base_frame = base_frame.split(SCOPE_DELIMITER)

        # If name and base_frame are scoped both should have a common scope
        split_frame_name = directive['name'].split(SCOPE_DELIMITER)
        if len(split_base_frame) > 1 and len(split_frame_name) > 1:
            if split_base_frame[:-1] != split_frame_name[:-1]:
                print(f'Frame named: [{directive["name"]}] has a different '
                      f'scope in its name and its base_frame: '
                      f'[{x_pf["base_frame"]}].')
                return False

        # Construct the necessary model scopes.
        current_model_scope = None
        model_root = root
        for scope in split_base_frame[:-1]:
            if current_model_scope is None:
                current_model_scope = scope
            else:
                current_model_scope += SCOPE_DELIMITER + scope

            # Check if the scope already exists, if it is change model_root
            # and move on.
            if current_model_scope in self.model_elem_map:
                model_root = self.model_elem_map[current_model_scope]
                continue

            # If not, we create it, save it to map and incomplete.
            new_model_root = ET.SubElement(model_root, 'model', name=scope)
            self.model_elem_map[current_model_scope] = new_model_root
            model_root = new_model_root

            # We only consider the most nested model scope as incomplete, since
            # all the parent model instances will be considered valid when the
            # nested model is valid.
            if current_model_scope is not None and \
                    current_model_scope not in self.incomplete_models:
                self.incomplete_models.append(current_model_scope)

        # Start constructing the frame in the model instance.
        translation_str = '0 0 0'
        if 'translation' in x_pf:
            trans = x_pf['translation']
            translation_str = f'{trans[0]} {trans[1]} {trans[2]}'

        # Supporting !Rpy degree rotations for now.
        # See drake/common/schema/transform.h.
        rotation_str = '0 0 0'
        if 'rotation' in x_pf:
            rot = x_pf['rotation']
            if rot['_tag'] != '!Rpy':
                print(f"Rotation of type {rot['_tag']} not suported.")
                return False
            rotation_str = f"{rot['deg'][0]} {rot['deg'][1]} {rot['deg'][2]}"

        frame_name = directive['name'].split(SCOPE_DELIMITER)[-1]
        frame_elem = ET.SubElement(model_root, 'frame', name=frame_name)

        pose_elem = ET.SubElement(
            frame_elem, 'pose', relative_to=split_base_frame[-1])
        pose_elem.text = f'{translation_str}   {rotation_str}'
        return True

    # For add_weld, we only support child frames that can be referred to
    # within the same sdformat file that are nested only once, as we would
    # require the placement_frame and pose combination for posturing.
    #
    # Supported example:
    # add_model:
    #   name: simple_model
    #   file: ...
    # add_weld:
    #   parent: ...
    #   child: simple_model::frame
    #
    # Unsupported example:
    # add_model:
    #   name: simple_model
    #   file: ...
    # add_weld:
    #   parent: ...
    #   child: simple_model::nested_model::frame
    #
    # Unsupported example:
    # add_directives:
    #   file: directive_that_contains_simple_model.yaml
    # add_weld:
    #   parent: ...
    #   child: simple_model::frame
    #
    # Whenever the top model instance in the child of the weld cannot be
    # referred to obviously, this conversion will fail. In the unsupported
    # example above, it is unclear where simple_model is located, hence the
    # placement_frame and pose combination cannot be applied for posturing.
    def add_weld_fixed_joint(
            self, root: ET, directive: Dict[str, Any]) -> bool:
        """Welding fixes two frames in the same global pose, and adds a fixed
        joint between them"""
        parent_name = directive['parent']
        # Resolve scope if the frame is implicit
        if len(directive['parent'].split(SCOPE_DELIMITER)) == 1:
            parent_name = self.resolve_and_scope_frame(parent_name)
            if parent_name is None:
                print(f'Failed trying to find scope for frame: '
                      f'[{directive["parent"]}]. When tring to add weld '
                      f'between: [{directive["parent"]}] and: '
                      f'[{directive["child"]}].')
                return False

        child_name = directive['child']
        if len(directive['child'].split(SCOPE_DELIMITER)) == 1:
            child_name = self.resolve_and_scope_frame(directive['child'])
            if child_name is None:
                print(f'Failed trying to find scope for frame: '
                      f'[{directive["child"]}]. When tring to add weld '
                      f'between: [{directive["parent"]}] and: '
                      f'[{directive["child"]}].')
                return False

        joint_name = \
            f'{parent_name.replace(SCOPE_DELIMITER, "__")}___to___' \
            f'{child_name.replace(SCOPE_DELIMITER, "__")}___weld_joint'

        joint_elem = ET.SubElement(root, 'joint', name=joint_name)
        joint_elem.set('type', 'fixed')

        parent_elem = ET.SubElement(joint_elem, 'parent')
        parent_elem.text = parent_name

        child_elem = ET.SubElement(joint_elem, 'child')
        child_elem.text = child_name

        # Bookkeeping for posturing models that are added later.
        split_child_name = child_name.split(SCOPE_DELIMITER)
        self.child_frame_model_name_to_weld_map[
            split_child_name[0]] = directive

        return True

    def convert_directive(self, input_path: str) -> str:
        # Initialize the sdformat XML root
        root = ET.Element('sdf', version=SDF_VERSION)
        root_model_name = os.path.splitext(os.path.basename(input_path))[0]
        root_model_elem = ET.SubElement(root, 'model', name=root_model_name)

        # Read the directives file
        directives = yaml_load_file(input_path)

        if list(directives.keys())[0] != 'directives':
            print('[directives] must be the first keyword in the yaml file,'
                  ' exiting.')
            return

        # Obtain the list of directives
        self.all_directives = directives['directives']

        # Construct all new model instances and start keeping track of welds
        # to be added.
        leftover_directives = []
        for directive in self.all_directives:
            if 'add_model_instance' in directive:
                new_model_name = directive['add_model_instance']['name']
                if new_model_name in self.model_elem_map or \
                        new_model_name in self.incomplete_models:
                    print(f'Model instance [{new_model_name}] already added')
                    return

                new_model_elem = ET.SubElement(root_model_elem, 'model',
                                               name=new_model_name)
                self.model_elem_map[new_model_name] = new_model_elem
                self.incomplete_models.append(new_model_name)

            elif 'add_weld' in directive:
                weld = directive['add_weld']
                weld_child = weld['child']
                weld_child_scopes = weld_child.split(SCOPE_DELIMITER)

                if len(weld_child_scopes) > 2:
                    print(
                        'Child frame in weld can only be nested in one model '
                        'for it to be postured correctly in a weld.')
                    return

                if not self.add_weld_fixed_joint(root_model_elem, weld):
                    print('Failed to convert add_weld directive.')
                    return

            else:
                leftover_directives.append(directive)

        # Next, frame elements and all their model instances will be
        # constructed.This cannot be done in the same loop as above since
        # we require all added model instances to be known.
        directives = leftover_directives
        leftover_directives = []
        for directive in directives:
            if 'add_frame' in directive:
                if not self.add_frame(root_model_elem, directive['add_frame']):
                    print('Failed to perform add_frame directive.')
                    return
            leftover_directives.append(directive)

        # Go through each directive add to XML as needed.
        directives = leftover_directives
        for directive in directives:
            success = True
            if 'add_model' in directive:
                success = self.add_model(
                    root_model_elem, directive['add_model'])

            elif 'add_directives' in directive:
                success = self.add_directives(
                    root_model_elem, directive['add_directives'])

            if not success:
                print(f'Failed to convert model directive {input_path}')
                return

        # Check if all the welds and incomplete model instances are taken
        # care of.
        if self.child_frame_model_name_to_weld_map:
            print('There are welds that are still not postured properly:')
            for key in self.child_frame_model_name_to_weld_map:
                print(f'    {self.child_frame_model_name_to_weld_map[key]}')
            return

        if len(self.incomplete_models) != 0:
            print('There are models that are generated but not used:')
            for m in self.incomplete_models:
                print(f'    {m}')
            return

        return ET.ElementTree(root)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '-m', '--model-directives', type=str,
        help='Path to model directives file to be converted.')
    parser.add_argument(
        '-o', '--output', type=str,
        help='Output path for converted sdformat file.')
    args = parser.parse_args()

    model_directives_to_sdf = ModelDirectivesToSdf()
    result_tree = model_directives_to_sdf.convert_directive(
        args.model_directives)

    if result_tree is not None:
        if args.output:
            result_tree.write(args.output,
                              pretty_print=True)
        else:
            print(ET.tostring(result_tree,
                              pretty_print=True,
                              encoding="unicode"))
    else:
        print('Failed to convert model directives.')
        return -1
    return 0


if __name__ == '__main__':
    sys.exit(main())