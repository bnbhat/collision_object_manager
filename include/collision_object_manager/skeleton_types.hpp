/**
 * @file skeleton_types.hpp
 * @author Balachandra Bhat (bnbhat311@gmail.com)
 * @brief contains type def for human skeletons from pose estimation
 * @version 1.0
 * @date 2023-09-05
 * 
 * @copyright Copyright (c) 2023
 * 
 */

/**
 * @brief Human skeleton formats
 * 
 */
enum class HUMAN_SKELETON_TYPE {
    BODY_18 = 18,  
    BODY_34 = 34,
    BODY_38 = 38,
};

/**
 * @brief COCOA18 format
 *       for more details visit https://www.stereolabs.com/docs/body-tracking/#how-it-works
 *          
 * 
 */
enum class BODY_18 {
    NOSE = 0,
    NECK = 1,
    RIGHT_SHOULDER = 2,
    RIGHT_ELBOW = 3,
    RIGHT_WRIST = 4,
    LEFT_SHOULDER = 5,
    LEFT_ELBOW = 6,
    LEFT_WRIST = 7,
    RIGHT_HIP = 8,
    RIGHT_KNEE = 9,
    RIGHT_ANKLE = 10,
    LEFT_HIP = 11,
    LEFT_KNEE = 12,
    LEFT_ANKLE = 13,
    RIGHT_EYE = 14,
    LEFT_EYE = 15,
    RIGHT_EAR = 16,
    LEFT_EAR = 17,
};


/**
 * @brief for more details visit https://www.stereolabs.com/docs/body-tracking/#how-it-works
 *          
 * 
 */
enum class BODY_34 {
    LEFT_SHOULDER = 5,
    LEFT_ELBOW = 6,
    LEFT_WRIST = 9,
    RIGHT_SHOULDER = 11,
    RIGHT_ELBOW = 13,
    RIGHT_WRIST = 16,
};

/**
 * @brief for more details visit https://www.stereolabs.com/docs/body-tracking/#how-it-works
 *          
 * 
 */
enum class BODY_38 {
    LEFT_SHOULDER = 12,
    LEFT_ELBOW = 14,
    LEFT_WRIST = 16,
    RIGHT_SHOULDER = 13,
    RIGHT_ELBOW = 15,
    RIGHT_WRIST = 17,
};
