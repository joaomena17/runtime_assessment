import yaml
from typing import List, Tuple
import sys

class RuntimeAssessmentConfig:
    """
    Class to load the configuration file for the runtime assessment.
    """
    def __init__(self, config_path: str = "specifications.yaml"):
        self.config_path = config_path
        self.config = self.parse_yaml_config(self.config_path)
        self.parse_setup()
        self.specifications = self.parse_specifications()


    def validate_config(self, config) -> None:
        """
        Validate the configuration file.
        :param config: dict
        :return: None
        """

        required_keys = ['setup', 'specifications']
        
        for key in required_keys:
            if key not in config:
                raise ValueError(f"Missing required key '{key}' in configuration")
        
        # TODO: try to make more thorough verification of specifications section


    def parse_yaml_config(self, yaml_file) -> dict:
        """
        Parse the YAML configuration file.
        :param yaml_file: str
        :return: dict
        """
        try:
            with open(yaml_file, 'r') as file:
                config = yaml.safe_load(file)

            self.validate_config(config)

            return config

        except FileNotFoundError:
            sys.exit(f"Configuration file not found: {yaml_file}")
        
        except Exception as e:
            sys.exit(f"Unexpected Error: {e}")


    def parse_setup(self) -> None:
        """
        Parse the setup parameters.
        :return: None
        """

        setup = self.config["setup"]

        if "target_node" not in setup:
            raise ValueError("Missing 'target_node' in setup")
        else:
            self.target_node = setup["target_node"]

        if "topics" not in setup:
            self.topics = []
        else:
            self.topics = setup["topics"]

        if "rate" not in setup:
            self.rate = 10
        else:
            self.rate = setup["rate"]
        
        if "logger_path" not in setup:
            self.logger_path = "log"
        else:
            self.logger_path = setup["logger_path"]


    def parse_specifications(self) -> dict:
        """
        Parse the specifications and output the requirements grouped by type of assessment, and for ros_topic assessments, grouped by topic.
        :return: dict
        """
        conf = self.config["specifications"]

        requirements = {"topic": {}, "metric": {}}

        for spec in conf:
            if "topic" in spec.keys():
                aux_dict = {}
                topic = spec["topic"]

                if topic not in requirements["topic"].keys():
                    requirements["topic"][topic] = []
                
                for field, value in spec.items():
                    if field != "topic":
                        aux_dict[field] = value

                requirements["topic"][topic].append(aux_dict)
            
            elif "metric" in spec.keys():
                aux_dict = {}
                metric = spec["metric"]

                if metric not in requirements["metric"].keys():
                    requirements["metric"][metric] = []

                for field, value in spec.items():
                    if field != "metric":
                        aux_dict[field] = value

                requirements["metric"][metric].append(aux_dict)
        
        return requirements