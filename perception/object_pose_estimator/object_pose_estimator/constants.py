# Camera intrinsics
FX = 525.0
FY = 525.0
CX = 160.0
CY = 120.0

# Kalman filter noise 
FPS                    = 30.0
PROCESS_NOISE_POSITION = 0.5    # m² per tick
PROCESS_NOISE_VELOCITY = 20     # (m/s)² per tick
MEASUREMENT_NOISE_POSITION = 0.005  # m²
OCCLUSION_R_SCALE      = 2000
VELOCITY_ZERO_THRESH   = 0.05   # m/s — zero velocity below this on OCC entry

# Depth Anything EMA scaling
DA_SCALE_EMA_ALPHA = 0.05
DA_SCALE_CLIP      = (0.5, 2.0)

# Rotation tracker 
ROTATION_PROCESS_NOISE_Q     = 1e-5
ROTATION_PROCESS_NOISE_W     = 1e-5
ROTATION_MEASUREMENT_NOISE   = 1.0
ANGULAR_VELOCITY_ZERO_THRESH = 0.05   # rad/s
ANGULAR_VELOCITY_MAX_THRESH  = 0.05   # rad/s

# Detection / segmentation models 
DETECTOR         = "gdino"
GDINO_MODEL      = "IDEA-Research/grounding-dino-tiny"
GDINO_BOX_THRESH = 0.5
SAM2_MODEL       = "facebook/sam2.1-hiera-base-plus"
DEPTH_MODEL      = "depth-anything/Depth-Anything-V2-Metric-Indoor-Small-hf"

# Tracking quality
MIN_SAM_SCORE    = 0.20
MIN_MASK_AREA_PX = 0

#To upscale/downscale image
SEGMENTATION_WIDTH  = 320
SEGMENTATION_HEIGHT = 240

OCC_CIRCLE_RADIUS = 30

# Occlusion thresholds
PARTIAL_OCCLUSION_THRESHOLD = 0.20  # arm covers >20% of bbox -> PARTIALLY_OCCLUDED
FULL_OCCLUSION_THRESHOLD    = 0.75  # arm covers >75% of bbox -> FULLY_OCCLUDED
MAX_OCCLUDED_AGE_S          = 5.0   # seconds before resetting a fully-occluded track

# Gripper detection
GRIPPER_GRASP_THRESHOLD = 0.02
GRIPPER_JOINT_NAMES = ['gripper_left_finger_joint', 'gripper_right_finger_joint']

# Robot geometry
ARM_LINK_NAMES = [
    'arm_1_link', 'arm_2_link', 'arm_3_link', 'arm_4_link',
    'arm_5_link', 'arm_6_link', 'arm_7_link',
    'hand_palm_link',
    'hand_left_finger_link', 'hand_right_finger_link',
]
ARM_LINK_RADIUS = 0.06
EEF_FRAME       = 'hand_palm_link'

# Camera
CAMERA_FRAME = 'xtion_rgb_optical_frame'

# Pipeline
ESTIMATION_RATE_HZ = 5.0
SERVICE_TIMEOUT_S  = 25.0
