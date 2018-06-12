int32 robotID
char symbolChr
int32 symbolIndex

# Keypoints
Keypoint[] keypoints

# Descriptors
sensor_msgs/Image desc

# BoW
uint32[] wordIds
float64[] wordValues
float32 minScore

# BoW-descriptor map
uint32[] nodeIds
Indices[] indicesVec

# Map points
uint32 nrMapPoints
uint32[] indices
float32[] worldPoints
float32[] maxDistInvariance
float32[] minDistInvariance
float32[] mfMaxDistance
sensor_msgs/Image pointDesc

# Pose
float32[] pose

# Calibration
float32[] K
float32 fx
float32 fy
float32 cx
float32 cy

#mvlevelsigma2
float32[] mvLevelSigma2
float32[] mvInvLevelSigma2
float32[] mvScaleFactors

# Misc
float32 mfLogScaleFactor
float32 mfGridElementWidthInv
float32 mfGridElementHeightInv
float32 mnMinX
float32 mnMinY
float32 mnMaxX
float32 mnMaxY
int32 mnScaleLevels
