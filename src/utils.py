import yaml
from typing import List, Tuple


def process_specification(specification_file: str) -> dict:
    with open(specification_file, 'r') as f:
        requirements = yaml.load(f)
    return requirements


def unordered_points(positions: List[Tuple], target: List[Tuple], tolerance: float) -> bool:
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


def get_average_value(record) -> float:
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