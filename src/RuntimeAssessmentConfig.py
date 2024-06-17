import yaml
from typing import List


class RuntimeAssessmentConfig:
    """
    Class to load the configuration file for the runtime assessment.
    """
    def __init__(self, config_path: str = "/home/joaomena/catkin_ws/src/runtime_assessment/src/config"):
        self.config_path = config_path
        self.config = self.parse_yaml_config(self.config_path)
        self.target_node = self.parse_target_node()
        self.topics = self.parse_topics()
        self.specifications = self.config["specifications"]


    def validate_config(self, config):
        """
        Validate the configuration file.
        :param config: dict
        :return: None
        """

        required_keys = ['setup', 'specifications']
        
        for key in required_keys:
            if key not in config:
                raise ValueError(f"Missing required key '{key}' in configuration")
        
        for spec in config['specifications']:
            required_spec_keys = ['name', 'type', 'params']
            for key in required_spec_keys:
                if key not in spec:
                    raise ValueError(f"Missing required key '{key}' in specification '{spec.get('name', '')}'")


    def parse_yaml_config(self, yaml_file):
        """
        Parse the YAML configuration file.
        :param yaml_file: str
        :return: dict
        """
        try:
            with open(yaml_file, 'r') as file:
                config = yaml.safe_load(file)

            self.validate_config(config)

            for spec in config['specifications']:
                for param, value in spec['params'].items():
                    if value == "~":
                        spec['params'][param] = None

            return config

        except FileNotFoundError:
            raise ValueError(f"Configuration file not found: {yaml_file}")
        except Exception as e:
            raise e


    def parse_target_node(self) -> str:
        """
        Parse the target node.
        :return: str
        """
        return self.config["setup"]["target_node"]
    

    def parse_topics(self) -> List[str]:
        """
        Parse the topics.
        :return: List[str]
        """
        return [t["topic"] for t in self.config["setup"]["topics"]]