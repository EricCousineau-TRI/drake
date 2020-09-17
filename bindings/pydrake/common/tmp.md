```sh
cd bindings/pydrake/common
( set -eux
python3 -m virtualenv -p python3 .venv/
set +eux
source .venv/bin/activate
set -eux
pip install ipywidgets==7.0.0 notebook packaging matplotlib
jupyter notebook ./jupyter_example.ipynb
)
```