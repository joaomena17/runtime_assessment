import yaml
from importlib import import_module
from typing import List, Tuple, Any, Union
import concurrent.futures


def check_value_params(value: float, target: Union[float, Tuple], tolerance: float = 0.05, comp: str = "=") -> bool:
    """
    Check if the average velocity of the turtle is within the tolerance of the target.
    :param value: float
    :param target: float || Tuple
    :param tolerance: float
    :return: bool
    """
    
    if isinstance(target, tuple):
        min_target, max_target = target
        if min_target < value < max_target:
            return True
        
    elif isinstance(target, float):
        comparison_dict = {
            "=": lambda: target*(1 - tolerance) < value < target*(1 + tolerance),
            ">": lambda: value > target,
            "<": lambda: value < target,
            ">=": lambda: value >= target,
            "<=": lambda: value <= target,
            "!=": lambda: value != target,
        }

        if comp in comparison_dict:
            return comparison_dict[comp]()

        else:
            raise ValueError(f"Invalid comparison operator: {comp}.")
    
    else:
        raise ValueError("Invalid target.")
        
    return False


def unordered_points(target: List[dict], record: List[Tuple], tolerance: float, timein=None, timeout=None) -> bool:
    """
    Check if certain positions occur in the recorded poses.
    :param positions: List[Tuple]
    :param target: List
    :param tolerance: float
    :return: bool
    """

    # look for every event in any order
    for pos in target:

        # check if the attributes are present in the recorded data
        for attr in list(pos.keys()):
            if not has_attribute(record[0][1], attr):
                raise ValueError(f"Attribute '{attr}' not found in record.")
        
        # default found state to false
        found = False

        # traverse the records and check if all attributes of the target's current position are present
        for element in record:
            _, data = element

            for attr, val in pos.items():
                # apply tolerance for numerical values
                if is_numeric(val):
                    check = abs(get_attribute(data, attr) - val) < (val * tolerance)
                else:
                    check = get_attribute(data, attr) == val
                
                if not check:
                    break

            if check:
                found = True
                break
                
        if not found:
            return False    
                
    return True


def ordered_points(target: List, record: List[Tuple], tolerance: float = 0.05, timein=None, timeout=None) -> bool:
    """
    Check if certain positions occur in the recorded poses in order.
    :param positions: List[Tuple]
    :param tolerance: float
    :return: bool
    """

    # copy the records and sort based on the first element of the timestamp
    records = sorted(record, key=lambda x: x[0])

    # look for the positions in order
    for pos in target:

        # check if the attributes are present in the recorded data
        for attr in list(pos.keys()):
            if not has_attribute(records[0][1], attr):
                raise ValueError(f"Attribute '{attr}' not found in record.")
            
        # default found state to false
        found = False

        # traverse the records and check if all attributes of the target's current position are present in order
        for i, element in enumerate(records):
            _, data = element

            for attr, val in pos.items():

                # apply tolerance for numerical values
                if is_numeric(val):
                    check = abs(get_attribute(data, attr) - val) < (val * tolerance)
                else:
                    check = get_attribute(data, attr) == val

            if check:
                # remove all positions that occured before the current one from search space
                records = records[i:]
                found = True
                break

        if not found:
            return False
    
    return True


def is_numeric(value: Any) -> bool:
    """
    Check if a value is numeric.
    :param value: Any
    :return: bool
    """
    return isinstance(value, (int, float))


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
    if len(events):
        total = 0
        for i in range(len(events) - 1):
            total += time_between_events(events[i], events[i + 1])
        return 1/(total / len(events))
    
    return 0


def get_average_value(attr: str, record: Tuple) -> float:
    """
    Update the average velocity.
    :return: float
    """
    if not record:
        raise ValueError("No records found.")
    
    if not has_attribute(record[0][1], attr):
        raise ValueError(f"Attribute '{attr}' not found in record.")
    
    if not is_numeric(get_attribute(record[0][1], attr)):
        raise ValueError(f"Attribute '{attr}' is not numeric.")

    total = 0
    for _, data in record:
        total += get_attribute(data, attr)

    return total / len(record)


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
        raise ValueError(f"Failed to import message type for topic '{topic}': {e}")
    

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


def get_attribute(obj, attribute):
    try:
        # Split the attribute by '.' to handle nested attributes
        attrs = attribute.split('.')
        value = obj
        for attr in attrs:
            value = getattr(value, attr)
        return value
        
    except AttributeError:
        return f"Attribute '{attribute}' not found in the object."


def filter_by_time(record: List[Tuple], timein=None, timeout=None) -> List[Tuple]:
    """
    Filter records based on the time range.
    :param record: List[Tuple]
    :param timein: float
    :param timeout: float
    :return: List[Tuple]
    """
    # filter the records based on the timein and timeout
    if timein is not None and timeout is not None:
        return [x for x in record if timein < x[0] < timeout]

    elif timein is not None:
        return [x for x in record if x[0] > timein]
    
    elif timeout is not None:
        return [x for x in record if x[0] < timeout]
    
    return record


def get_max(attr: str, record: List[Tuple]) -> float:
    """
    Get the maximum value from the record.
    :param record: List[Tuple]
    :return: float
    """
    if not record:
        raise ValueError("No records found.")
    
    if not has_attribute(record[0][1], attr):
        raise ValueError(f"Attribute '{attr}' not found in record.")

    if not is_numeric(get_attribute(record[0][1], attr)):
        raise ValueError(f"Attribute '{attr}' is not numeric.")
        
    return max([get_attribute(x[1], attr) for x in record])


def get_min(attr: str, record: List[Tuple]) -> float:
    """
    Get the minimum value from the record.
    :param record: List[Tuple]
    :return: float
    """
    if not record:
        raise ValueError("No records found.")
    
    if not has_attribute(record[0][1], attr):
        raise ValueError(f"Attribute '{attr}' not found in record.")
    
    if not is_numeric(get_attribute(record[0][1], attr)):
        raise ValueError(f"Attribute '{attr}' is not numeric.")
    
    return min([get_attribute(x[1], attr) for x in record])