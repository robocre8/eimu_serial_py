- create virtual env and activate it
  ```shell
    python3 -m venv .env
  ```
  ```shell
    python3 -m venv .env
  ```

- install necessary packges for build
  ```shell
    python3 -m pip install -r requirements.txt
  ```

- start build
  ```shell
    python3 -m build
  ```

- push to PYpi
  ```shell
    twine upload dist/* #PyPI
  ```
  OR
  ```shell
    python3 -m twine upload --repository testpypi dist/* #testPyPI
  ```

- remove dist
  ```shell
    rm -rf ./dist ./eimu_serial.egg-info
  ```