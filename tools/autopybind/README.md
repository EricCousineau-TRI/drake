# Automated Python Binding Generator

This provides an integration with
[`autopybind11`](https://gitlab.kitware.com/autopybind11/autopybind11). At present, these are meant to *bootstrap* bindings
(e.g. generate or regenerate bindings for a class), but are not meant to
completely replace the manual binding process.

## Running

To output template bindings to `/tmp/autopybind11`, please run the following:

```sh
bazel run //tools/autopybind:generate -- --output_dir=/tmp/autopybind11
```

This is configured using the `bindings_to_generate.yaml` file in this
directory. For more information on the `autopybind11` schema, please review its 
documentation:
<br/>
<https://gitlab.kitware.com/autopybind11/autopybind11>

## Troubleshooting

If you encounter an issue w/ our usage of `autopybind11`, please run the
following script which captures all output, commit the generate files to git,
and make a draft PR pointing to what problem you're having:

```sh
cd drake
./tools/autopybind/generate_debug.sh
```

## Example Workflows

With inline pain points

### For `RenderEngineGlParams`

- Pinpoint symbol, and restructure (redundant) information between file and
  namespacing.
- Run `generate` binary as instructed above.
- Replace `RenderEngineGlParams` with `Class` alias (per pydrake docs)
- Ignore styling; assume Drake's `clang-format` settings + lint will fix it
- For `.def(py::init<Class const &>(), py::arg("arg0"))`:
    - Change `arg0` to `other` (user-friendly name)
- Replace `py::init<>()` with `py::init()`
- Put `py::init<Class const &>()` after `py::init()`
- Add in `constexpr auto& cls_doc = doc.RenderEngineGlParams`
    - Naively insert relevant `cls_doc` (will let compiler errors tell me which
      doc symbols I should look up)
- Copy code, paste into relevant section (`geometry_py.cc`)
- Reformat:
  `/usr/bin/clang-format-9 -style=file -i bindings/pydrake/geometry_py.cc`
- `Class const &` was not reversed.
- Additional lint error:
  `bindings/pydrake/geometry_py.cc:427:  Line contains only semicolon. If this should be an empty statement, use {} instead.  [whitespace/semicolon] [5]`
    - Manually fix lint error
- Build: `bazel build //bindings/pydrake:geometry_py`
  - Get compiler error about `cls_doc.ctor.doc`
  - Do documented steps to find symbols:
    ```
    $ bazel build //bindings/pydrake:documentation_pybind.h
    $ $EDITOR bazel-bin/bindings/pydrake/documentation_pybind.h
    ```
  - Search for `RenderEngineGlParams::RenderEngineGlParams`
  - Realize it was actually just a param struct (no ctor); remove `cls_doc.ctor.doc*`.
  - Because it is a param struct, replace construcotrs with `ParamInit<Class>()`
- Because this is consumed by `MakeRenderEngineGl`, change
  `bindings_to_generate.yaml` to only generate for this symbol.
    - Requires me to know that it is a function, changing fields, and learning
      more about config. I guess `functions` is the right key.
    - Rerun `generate`
    - I get a new error: `IndexError: list index out of range`
    - I try scrolling up, but I see 100's of warnings about files not existing.
      See `tmux capture-pane` output: https://gist.github.com/EricCousineau-TRI/95bb4970d1d38d552baff60fc6a97a91
    - Two errors reported on lines 956 and 1990. They seem duplicated, but
      unclear.
    - I assume that `autopybind11` does not validate schema, so maybe
      `functions` is incorrect.
    - I visit `autopyinbd11` README, hoping that I'm using same version as what
      Drake has.
    - I see that `functions` is indeed correct.
    - I go back to the file, cross reference binding file, and realize I
      included the same file. (redundant information that hurts).
    - I fix the file path to use `render_engine_gl_factory.h`
    - Succeeds, I think. It runs in 6s, with tons of warnings.
    - I look at generated files in this order (based on my guesses) in
      `/tmp/output.txt`:
        - `pydrake_free_functions_py.cpp`
        - `wrapper_pydrake.cpp`
        - `pydrake.cpp`
        - `drake_py.cpp`
        - `geometry_py.cpp`
        - `render_py.cpp` - Ah, this is the one it has.
    - I see the `Pyrender.def` usage. I copy the code.
    - I paste it above existing definition for `MakeRenderEngineGl`
    - I replace `Pyrender` with `m`
    - I strip leading `::`
    - I delete `drake::geometry::render`
    - I delete `default_delete`
    - I copy `doc.MakeRenderEngineGl.doc` to newly generated version.
    - I change `RenderEngineGlParams({})` to `RenderEngineGlParams()`
    - I delete old definition for `MakeRenderEngineGl`
    - Reformat: `/usr/bin/clang-format-9 -style=file -i bindings/pydrake/geometry_py.cc`
    - Build anew `bazel build //bindings/pydrake:geometry_py`
- It builds, now I write unittests.

### For `StaticEquilibriumConstraint`

- Clean output directory: `rm -rf /tmp/autopybind11/`
- Adjust `bindings_to_generate.yaml` for only this symbol
- Review autopybind11 README
- Try out `ignore_namespace_structure: True` to simplify finding files
- Review template text blocks
  - File: <https://gitlab.kitware.com/autopybind11/autopybind11/-/merge_requests/170>
  - Was going to customize copy ctor, but realized `copy_constructor_tramp` was
    only for copy ctors.
- Try running, but get an error,
  `unrecognized arguments: --ignore_namespace_structure=True`
- Searching `autopybind11@2982f41`, seems like this isn't actually an option?
  (it's only in README)
  - Figure out when it got changed: 
    ```
    cd autopybind11
    git log -n 1 -Signore_namespace_structure
    gitk e9849032e3919e2e68f9e2032b2bb3b183e5939e
    ```
  - It's actually `enforce_namespace_structure`
  - Do git forensics, confirm that it's now negated
  - Submit PR:
    <https://gitlab.kitware.com/autopybind11/autopybind11/-/merge_requests/171>
- Run `generate` (takes 45s), see warnings:
  ```
  Warning: Class Constraint  was not found in current module or any linked module
  Warning: Class EvaluatorBase  was not found in current module or any linked module
  Warning: Class StaticEquilibriumConstraint will not have inheritance relationship with class Constraint in binding code.
  ```
  - Decide to bind `Constraint`; will ignore those bindings, but I want
    inheritance.
  - Do sleuthing to identify correct redundant information.
  - Assume it won't hurt if `EvaluatorBase` isn't bound.
  - Delete output dir again
- Run `generate` again (47s)
  - Look directly at `StaticEquilibriumConstraint_py.cpp`
  - Only `contact_pair_to_wrench_evaluator` is bound; `MakeBinding` is not
    bound.
- Review `autopybind11/README` if there's an obvious mention of `static`. It
  isn't mentioned. Assume that I need to do
  `classes: { my_class: { functions: { my_static_method: {}}}}`
- Run `generate` again (52s)
  - It still doesn't appear?
  - See warning
    ```
    Warning member MakeBinding will not be bound due to unmet dependency Binding<drake::multibody::StaticEquilibriumConstraint>
    ```
  - Realize this warning was actually there without `my_static_method` being
    added. Remove it. (Pain point: no validation)
  - Add instantiation:
    ```
    Binding:
      inst: ["drake::multibody::StaticEquilibriumConstraint"]
    ```
  - See: `Unable to locate class definition.`
  - Go back and use correct headers file, copy and paste nested YAML structure
  - Accidentally opened file, and saw that `MakeBinding` was there? (Perhaps
    the "Unable to locate class definition" was for something else?)
- Take generated bindings, and reformat:
  - Replace `StaticEquilibriumConstraint` w/ `Class` typedef
  - Delete `::drake::solvers`
  - Get overwhelmed by noisy signatures; look at C++ header file instead
  - Copy+paste from C++ header file, re-massage.
  - Fix brackets and whatever.
  - Add `constexpr auto& cls_doc` thing
  - Start reformatting, but realized I forgot to configure `keep_alive` in
    autopybind11 config. It's easier for me to just do it myself, so I do it
    myself in binding code.
  - Reformat:
    `/usr/bin/clang-format-9 -style=file -i bindings/pydrake/multibody/optimization_py.cc`
  - Try building, realize that `GeometryPairContactWrenchEvaluatorBinding` is internal (our API mistake)
  - To workaround, I can decompose it in Python and return tuple. 
  - I now need to write this as a lambda function and change everything.
  - Er, no, maybe write `type_caster`.
  - Er, now getting compiler errors. Giving up.
