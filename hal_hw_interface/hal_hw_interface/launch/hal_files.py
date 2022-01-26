import os
import yaml
import pathlib
import subprocess

from launch import logging
from launch.utilities import ensure_argument_type

from launch_ros.utilities import normalize_parameters, evaluate_parameters

from .hal_ordered_action import HalOrderedAction


class HalFiles(HalOrderedAction):
    """Load HAL files and pass parameters."""

    def __init__(
        self,
        *,
        hal_name="hal_files",
        hal_file_dir=None,
        hal_files=None,
        parameters=None,
        **kwargs,
    ):
        """
        Load a HAL file.

        :param: hal_file_dir  HAL file directory
        :param: hal_files  List of hal files to load from hal_file_dir
        :param: parameters list of names of yaml files with parameter values,
            or dictionaries of parameters.

        Parameters are handled similar to the Node launch entity.  The
        parameters are passed as a list, with each element either a
        yaml file that contains parameter rules (string or
        pathlib.Path to the full path of the file), or a dictionary
        that specifies parameter rules.

        Parameters are parsed and added as the global `parameters` to
        the HAL file python environment.
        """
        super().__init__(hal_name=hal_name, **kwargs)
        self.__logger = logging.get_logger(__name__)
        assert (
            hal_file_dir is not None
        ), "Missing 'hal_file_dir' launch argument"
        self.__hal_file_dir = hal_file_dir
        self.__hal_files = hal_files
        if parameters is not None:
            ensure_argument_type(parameters, (list), "parameters", "Node")
            # All elements in the list are paths to files with
            # parameters (or substitutions that evaluate to paths), or
            # dictionaries of parameters (fields can be
            # substitutions).
            normalized_params = normalize_parameters(parameters)
        self.__parameters = normalized_params or []

    def parse_parameters(self, context):
        res = dict()
        # Expand parameters
        # ...cribbed from launch_ros/actions/node.py
        evaluated_parameters = evaluate_parameters(context, self.__parameters)
        for params in evaluated_parameters:
            if isinstance(params, dict):
                res.update(params)
            elif isinstance(params, pathlib.Path):
                with open(str(params), "r") as f:
                    data = yaml.safe_load(f)
                assert isinstance(
                    data, dict
                ), f"HAL file params YAML must be a mapping:  '{str(params)}'"
                res.update(data)
            else:
                raise RuntimeError("invalid parameters {repr(params)}")
        return res

    def execute_deferred(self, context):
        # Parse parameters
        parameters = self.parse_parameters(context)

        # Expand and check the hal file directory and hal file list
        hal_file_dir = context.perform_substitution(self.__hal_file_dir)
        hal_files = [context.perform_substitution(i) for i in self.__hal_files]
        assert os.path.exists(
            hal_file_dir
        ), f"HAL file directory doesn't exist:  '{hal_file_dir}'"
        for hal_file in hal_files:
            # Construct and check HAL file path
            hal_file_path = os.path.join(hal_file_dir, hal_file)
            assert os.path.exists(
                hal_file_path
            ), f"HAL file path doesn't exist:  '{hal_file_path}'"

            self.__logger.info(f"Loading HAL file '{hal_file_path}'")
            if hal_file_path.endswith(".py"):
                with open(hal_file_path, "r") as f:
                    data = compile(f.read(), hal_file_path, "exec")
                globals_ = dict(parameters=parameters)
                exec(data, globals_)
            else:
                subprocess.check_call(["halcmd", "-f", hal_file_path])

        self.__logger.info("HAL files loaded")
