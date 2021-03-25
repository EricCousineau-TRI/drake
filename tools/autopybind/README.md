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

## Example Workflow

TODO(eric): Add this in.

## Troubleshooting

If you encounter an issue w/ our usage of `autopybind11`, please run the
following script which captures all output, commit the generate files to git,
and make a draft PR pointing to what problem you're having:

```sh
cd drake
./tools/autopybind/generate_debug.sh
```

## Steps Taken

- For `RenderEngineGlParams`:
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
