import yaml
from typing import List, Tuple


class RuntimeAssessmentConfig:
    """
    Class to load the configuration file for the runtime assessment.
    """
    def __init__(self, config_path: str = "specifications.yaml"):
        self.config_path = config_path
        self.config = self.parse_yaml_config(self.config_path)
        self.parse_setup()
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
            print(spec)
            valid_spec_keys = ['metric_assessment', 'ros_topic_assessment']
            if spec not in valid_spec_keys:
                raise ValueError(f"Key {spec} is not a valid specification declaration.")


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

            return config

        except FileNotFoundError:
            raise ValueError(f"Configuration file not found: {yaml_file}")
        
        except Exception as e:
            raise e


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
        return self.config["setup"]["topics"]
    

    def parse_specifications(self):
        """
        Parse the specifications.
        :return: dict
        """
        for spec in self.config["specifications"]:
            yield spec
