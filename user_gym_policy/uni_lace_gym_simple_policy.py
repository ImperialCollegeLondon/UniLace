import numpy as np
import yaml
from uni_lace.uni_lace_gym_env import UniLaceGymEnv


# observe pose
test_action_l = [-0.720012903213501, -0.3579239249229431, 1.772327184677124, 0.13136780261993408, 
                    -0.648350715637207, 1.1232486963272095, -1.8344221115112305]
test_action_r = [0.8814099431037903, -0.36250463128089905, -1.85231351852417, -0.028060272336006165, 
                    0.606028139591217, 1.1636393070220947, 1.9661468267440796]
# # grasp pose
# test_action_l = [-1.0771714448928833, -0.7815413475036621, 1.3724327087402344, 0.3715556263923645, 
#                  1.7982889413833618, 0.8510035872459412, 2.2453041076660156]
# test_action_r = [1.0602233409881592, -0.6867497563362122, -1.4237779378890991, 0.3548184633255005, 
#                  4.543345928192139, 0.7604496479034424, -2.2329251766204834]

if __name__ == '__main__':
    with open('/UniLace/user_gym_policy/param_gym.yaml') as f:
        params = yaml.load(f, Loader=yaml.FullLoader)
    env = UniLaceGymEnv(params=params)

    # test the step function
    input('Press Enter to move the robot')
    obs, reaward, done, info = env.step(test_action_l, test_action_r, 0, 1, 1, 1)
    # print all images
    import cv2
    for img in obs['rgbd_images']:
        if img['rgb'] is not None:
            img['rgb'] = cv2.resize(img['rgb'], (640, 360)) # resize all images to (360, 640)
        if img['depth'] is not None:
            img['depth'] = cv2.resize(img['depth'], (640, 360))
    # combine all images into one
    img = np.zeros((360*2, 640*len(obs['rgbd_images']), 3), dtype=np.uint8)
    for i, img_data in enumerate(obs['rgbd_images']):
        if img_data['rgb'] is not None:
            img[0:360, i*640:(i+1)*640, :] = img_data['rgb']
        if img_data['depth'] is not None:
            img_data['depth'] = cv2.normalize(img_data['depth'], None, 0, 255, cv2.NORM_MINMAX)
            img[360:, i*640:(i+1)*640, :] = cv2.cvtColor(img_data['depth'], cv2.COLOR_GRAY2BGR)
    from matplotlib import pyplot as plt
    plt.imshow(img)
    plt.show()

    # test the reset function
    input('Press Enter to reset the environment')
    env.reset()

    env.close()