## ObjectScore.msg
## Represents an object with its detection info and scoring

# Object identification
string label                      # Object class name from YCB dataset
int32 object_id                   # Unique ID for tracking
string color                      # Color category (for bin sorting)

# Detection info
sensor_msgs/RegionOfInterest roi  # Bounding box in image
geometry_msgs/Point position      # 3D position in world frame
float32 confidence                # Detection confidence [0-1]

# Scoring components
float32 base_score                # Base score from competition rules
float32 distance                  # Distance to robot (meters)
float32 grasp_difficulty          # Estimated grasp difficulty [0-1]

# Computed priority
float32 total_score               # Weighted final score for decision making
