import yaml
from importlib import import_module
from typing import List, Tuple, Any


def unordered_points(positions: List[Tuple], target: List[Tuple], tolerance: float) -> bool:
    """
    Check if certain positions occur in the recorded poses.
    :param positions: List[Tuple]
    :param target: List[Tuple]
    :param tolerance: float
    :return: bool
    """

    for pos in positions:
        tgt_x, tgt_y = pos
        found = False
        for element in target:
            _, pose = element
            if abs(pose.x - tgt_x) < (tgt_x * tolerance) and abs(pose.y - tgt_y) < (tgt_y * tolerance):
                found = True
                break
        if not found:
            return False    
                
    return True


def ordered_points(positions: List[Tuple], target: List[Tuple], tolerance: float = 0.05) -> bool:
    """
    Check if certain positions occur in the recorded poses in order.
    :param positions: List[Tuple]
    :param tolerance: float
    :return: bool
    """
    # copy the records and sort based on the first element of the timestamp
    records = sorted(target, key=lambda x: x[0])

    # look for the positions in order
    for pos in positions:
        tgt_x, tgt_y = pos
        found = False

        for i, element in enumerate(records):
            _, pose = element
            if abs(pose.x - tgt_x) < (tgt_x * tolerance) and abs(pose.y - tgt_y) < (tgt_y * tolerance):                    
                # remove all positions that occured before the current one from search space
                records = records[i:]
                found = True
                break
            else:
                found = False

        if not found:
            return False
    
    return True


def time_between_events(e1: Tuple, e2: Tuple) -> float:
    """
    Calculate the time between two events.
    :param e1: Tuple
    :param e2: Tuple
    :return: float
    """
    return abs(e1[0] - e2[0])


def frequency_of_events(events: List[Tuple]) -> float:
        """
        Calculate the frequency of events.
        :param events: List[Tuple]
        :return: float
        """
        if events:
            total = 0
            for i in range(len(events) - 1):
                total += time_between_events(events[i], events[i + 1])
            return 1/(total / len(events))
        
        return 0


def get_average_value(record: Tuple) -> float:
        """
        Update the average velocity.
        :return: float
        """
        if record:
            total = 0
            for element in record:
                _, vel = element
                total += vel.linear.x

            return total / len(record)
        
        raise ValueError("No velocity records found.")


def import_message_type(topic_tuple: Tuple[str, str]) -> Any:
    """
    Import message types from a dictionary of topic names to message types
    :param topic_types: A tuple mapping topic names to message types
    :return: Imported class 
    """
    topic, msg_type = topic_tuple

    package_name, message_name = msg_type.split('/')
    package_name = package_name + '.msg'

    try:
        message_module = import_module(package_name)
        message_class = getattr(message_module, message_name)
        return message_class
    
    except (AttributeError, ModuleNotFoundError) as e:
        print(f"Failed to import message type for topic '{topic}': {e}")
        return None
    

def has_attribute(obj: Any, attribute_path: str) -> bool:
    """
    Check if an object has an attribute (implicit or nested)
    :param obj: Object to check
    :param attribute_path: Path to the attribute
    :return: True if the object has the nested attribute, False otherwise
    """
    current_object = obj
    for attr in attribute_path.split('.'):
        if not hasattr(current_object, attr):
            return False
        current_object = getattr(current_object, attr)
    return True    
