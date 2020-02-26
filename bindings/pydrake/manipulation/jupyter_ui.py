
class PlantJointSlider(object):
    """
    Example:

        def update(context, model_q_pairs):
            print(model_q_pairs)

        slider = PlantJointSlider(plant, update, [iiwa])
        display(slider.get_widget())
    """
    def __init__(
            self, plant, models, update_callback, context=None,
            continuous_update=False,
            num_steps=100):
        self._update_callback = update_callback
        nq = plant.num_positions()
        if context is None:
            context = plant.CreateDefaultContext()
        self._context = context

        class Model:
            def __init__(self, model):
                self.model = model
                self.name = plant.GetModelInstanceName(model)
                nq = plant.num_positions(model)
                self.qs = plant.GetPositions(context, self.model)
                self.joints = []
                self._index_map = dict()

            def add_joint(self, joint):
                self._index_map[joint] = len(self.joints)
                self.joints.append(joint)

            def set(self, joint, q):
                i = self._index_map[joint]
                self.qs[i] = q

            def get(self, joint):
                i = self._index_map[joint]
                return self.qs[i]

            def read(self):
                self.qs[:] = plant.GetPositions(context, self.model)

            def write(self):
                plant.SetPositions(context, self.model, self.qs)

        class JointInfo:
            def __init__(self, model, widget):
                self.model = model
                self.widget = widget

        self._joint_info = dict()
        self._models = [Model(x) for x in models]
        model_map = {int(x.model): x for x in self._models}

        joints = []
        for index in range(plant.num_joints()):
            joint = plant.get_joint(JointIndex(index))
            model = model_map.get(int(joint.model_instance()))
            if model is None:
                continue
            if joint.num_positions() == 0:
                continue
            assert joint.num_positions() == 1
            model.add_joint(joint)
            self._joint_info[joint] = JointInfo(model, None)

        def joint_callback(joint, value):
            model = self._joint_info[joint].model
            model.set(joint, value["new"])
            self._update()

        # Setup widgets
        joint_widgets = []
        rows = []
        for model in self._models:
            for joint in model.joints:
                step = (joint.position_upper_limits() -
                        joint.position_lower_limits()) / num_steps
                joint_widget = widgets.FloatSlider(
                    value=model.get(joint),
                    min=joint.position_lower_limits(),
                    max=joint.position_upper_limits(),
                    step=step,
                    continuous_update=continuous_update,
                )
                joint_widget.observe(partial(joint_callback, joint), "value")
                joint_widgets.append(joint_widget)
                self._joint_info[joint].widget = joint_widget
                name = model.name + "::" + joint.name()
                label = widgets.Label(name, layout={"width": "300px"})
                rows.append(widgets.HBox([label, joint_widget]))

        self.widget = widgets.VBox(
            rows,
            layout=widgets.Layout(min_width="800px"),
        )
        self._update()

    def _update(self):
        for model in self._models:
            model.write()
        self._update_callback(self._context, self.model_q_pairs)

    @property
    def model_q_pairs(self):
        """Returns a list of tuples, (name, q), for each model intsance
        provided in the constructor."""
        model_q_pairs = list()
        for model in self._models:
            model_q_pairs.append([model.name, model.qs.tolist()])
        return model_q_pairs

    @model_q_pairs.setter
    def model_q_pairs(self, model_q_pairs):
        model_q_pairs = dict(model_q_pairs)
        for model in self._models:
            qs = model_q_pairs.get(model.name)
            if qs is None:
                # Use model instance.
                qs = model_q_pairs.get(model.model)
            if qs is None:
                continue
            model.qs[:] = model_q_pairs[model.name]
            for joint in model.joints:
                widget = self._joint_info[joint].widget
                widget.value = model.get(joint)
        self._update()

    def update_from_context(self):
        for model in self._models:
            model.read()
        self.model_q_pairs = dict()
