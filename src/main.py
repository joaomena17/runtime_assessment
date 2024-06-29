from RuntimeAssessmentConfig import RuntimeAssessmentConfig
from RuntimeAssessment import RuntimeAssessment

def main():
    conf = RuntimeAssessmentConfig(config_path="/home/joaomena/catkin_ws/src/runtime_assessment/src/specifications.yaml")
    runtime_assessment = RuntimeAssessment()
    runtime_assessment.run()

if __name__ == "__main__":
    main()