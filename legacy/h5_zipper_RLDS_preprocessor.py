import os
import glob
import h5py
import numpy as np
from tqdm import tqdm
from PIL import Image

def process_episodes(input_dir, output_dir, action_key):
    os.makedirs(output_dir, exist_ok=True)
    episode_paths = sorted(glob.glob(os.path.join(input_dir, "episode_*.h5")))
    total_steps = len(episode_paths) * 654

    with tqdm(total=total_steps) as pbar:
        for episode_path in episode_paths:
            episode_name = os.path.splitext(os.path.basename(episode_path))[0]
            image_folder = os.path.join(input_dir, f"{episode_name}_images")

            with h5py.File(episode_path, "r") as h5f:
                action = h5f[action_key][:]
                # Fix actions of length 4 to length 7 as [a0,a1,a2,0,0,0,a3]
                fixed_actions = []
                for a in action:
                    if a.shape[0] == 4:
                        a = np.array([a[0], a[1], a[2], 0, 0, 0, a[3]])
                    fixed_actions.append(a)
                action = np.array(fixed_actions)                
                robot_state = h5f["robot_state"][:]
                episode_info = h5f["episode_info"][:]

            image_files = sorted(glob.glob(os.path.join(image_folder, "user_*.jpg")))
            wrist_image_files = sorted(glob.glob(os.path.join(image_folder, "wrist_*.jpg")))


            if not (len(image_files) == len(wrist_image_files) == action.shape[0] == robot_state.shape[0]):
                print(f"\nMismatch in episode: {episode_name}")
                print(f"Images: {len(image_files)}, Wrist Images: {len(wrist_image_files)}, Actions: {action.shape[0]}, States: {robot_state.shape[0]}")
                continue

            images = []
            wrist_images = []

            for img_path, wrist_path in zip(image_files, wrist_image_files):
                image = np.array(Image.open(img_path).resize((640, 640)), dtype=np.uint8)
                wrist_image = np.array(Image.open(wrist_path).resize((640, 640)), dtype=np.uint8)
                images.append(image)
                wrist_images.append(wrist_image)
                pbar.update(1)

            if len(images) == 0 or len(wrist_images) == 0:
                print(f"Skipping {episode_name} — no valid image data found.")
                continue

            output_path = os.path.join(output_dir, f"{episode_name}.h5")
            with h5py.File(output_path, "w") as out:
                out.create_dataset("action", data=action, dtype=np.float32)
                out.create_dataset("robot_state", data=robot_state, dtype=np.float32) # <- omit robot_state for no proprioception
                out.create_dataset("episode_info", data=episode_info)
                out.create_dataset("image", data=np.stack(images), dtype=np.uint8)
                out.create_dataset("wrist_image", data=np.stack(wrist_images), dtype=np.uint8)


if __name__ == "__main__":
    # input_dir = "/home/demoaccount/Data/demoaccount2/robot_manipulation/autogen_AB_random_large_5hz_with_pad/train"   # directory with episode_*.h5 and episode_*_images/
    # output_dir = "/home/demoaccount/Data/demoaccount2/rlds_dataset_builder/autogen_AB_random_large_5hz_with_pad_js/data/train"      # directory to write zipped h5s compatible with RLDS
    # process_episodes(input_dir, output_dir, action_key="action")

    # input_dir = "/home/demoaccount/Data/demoaccount2/robot_manipulation/autogen_AB_random_large_5hz_with_pad/train"   # directory with episode_*.h5 and episode_*_images/
    # output_dir = "/home/demoaccount/Data/demoaccount2/rlds_dataset_builder/autogen_AB_random_large_5hz_with_pad_eef/data/train"      # directory to write zipped h5s compatible with RLDS
    # process_episodes(input_dir, output_dir, action_key="action_tcp")
    
    
    # input_dir = "/media/Data/demoaccount2/robot_manipulation/first_test_haptic/train"   # directory with episode_*.h5 and episode_*_images/
    # output_dir = "/media/Data/demoaccount2/robot_manipulation/first_test_haptic/conversion_ready_new"      # directory to write zipped h5s compatible with RLDS
    # process_episodes(input_dir, output_dir, action_key="action") 

    input_dir = "/media/Data/demoaccount2/robot_manipulation/data_with_fts/train"   # directory with episode_*.h5 and episode_*_images/
    output_dir = "/media/Data/demoaccount2/robot_manipulation/data_with_fts/preprocessed"      # directory to write zipped h5s compatible with RLDS
    process_episodes(input_dir, output_dir, action_key="action") 





    # PAPER EXTENSION DATASETS --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

    # LIGHT TASK
    # input_dir1 = "/media/Data/demoaccount2/robot_manipulation/paper_extension_data/light/haptic/train"   
    # output_dir1 = "/media/Data/demoaccount2/robot_manipulation/paper_extension_data/conversion_ready/with_proprio/light_haptic"  
    # process_episodes(input_dir1, output_dir1, action_key="action_tcp") 

    # input_dir2 = "/media/Data/demoaccount2/robot_manipulation/paper_extension_data/light/vr/train"   
    # output_dir2 = "/media/Data/demoaccount2/robot_manipulation/paper_extension_data/conversion_ready/with_proprio/light_vr" 
    # process_episodes(input_dir2, output_dir2, action_key="action_tcp") 

    #input_dir3 = "/media/Data/demoaccount2/robot_manipulation/paper_extension_data/light/mixed/train"   
    #output_dir3 = "/media/Data/demoaccount2/robot_manipulation/paper_extension_data/conversion_ready/with_proprio/light_mixed"    
    #process_episodes(input_dir3, output_dir3, action_key="action_tcp") 

    # input_dir4 = "/media/Data/demoaccount2/robot_manipulation/paper_extension_data/light/haptic/train"   
    # output_dir4 = "/media/Data/demoaccount2/robot_manipulation/paper_extension_data/conversion_ready/without_proprio/light_haptic"  
    # process_episodes(input_dir4, output_dir4, action_key="action_tcp") 

    # input_dir5 = "/media/Data/demoaccount2/robot_manipulation/paper_extension_data/light/vr/train"   
    # output_dir5 = "/media/Data/demoaccount2/robot_manipulation/paper_extension_data/conversion_ready/without_proprio/light_vr" 
    # process_episodes(input_dir5, output_dir5, action_key="action_tcp") 

    # input_dir6 = "/media/Data/demoaccount2/robot_manipulation/paper_extension_data/light/mixed/train"   
    # output_dir6 = "/media/Data/demoaccount2/robot_manipulation/paper_extension_data/conversion_ready/without_proprio/light_mixed"    
    # process_episodes(input_dir6, output_dir6, action_key="action_tcp") 



    # WIPING TASK
    # input_dir7 = "/media/Data/demoaccount2/robot_manipulation/paper_extension_data/wiping/haptic/train"   
    # output_dir7 = "/media/Data/demoaccount2/robot_manipulation/paper_extension_data/conversion_ready/with_proprio/wiping_haptic"  
    # process_episodes(input_dir7, output_dir7, action_key="action_tcp") 

    # input_dir8 = "/media/Data/demoaccount2/robot_manipulation/paper_extension_data/wiping/vr/train"   
    # output_dir8 = "/media/Data/demoaccount2/robot_manipulation/paper_extension_data/conversion_ready/with_proprio/wiping_vr" 
    # process_episodes(input_dir8, output_dir8, action_key="action_tcp") 
    
    # input_dir9 = "/media/Data/demoaccount2/robot_manipulation/paper_extension_data/wiping/mixed/train"   
    # output_dir9 = "/media/Data/demoaccount2/robot_manipulation/paper_extension_data/conversion_ready/with_proprio/wiping_mixed"    
    # process_episodes(input_dir9, output_dir9, action_key="action_tcp") 

    #input_dir10 = "/media/Data/demoaccount2/robot_manipulation/paper_extension_data/wiping/haptic/train"   
    #output_dir10 = "/media/Data/demoaccount2/robot_manipulation/paper_extension_data/conversion_ready/without_proprio/wiping_haptic" 
    #process_episodes(input_dir10, output_dir10, action_key="action_tcp")  

    # input_dir11 = "/media/Data/demoaccount2/robot_manipulation/paper_extension_data/wiping/vr/train"   
    #output_dir11 = "/media/Data/demoaccount2/robot_manipulation/paper_extension_data/conversion_ready/without_proprio/wiping_vr" 
    # output_dir11 = "/media/Data/demoaccount2/rlds_dataset_builder/wiping_vr/"
    # process_episodes(input_dir11, output_dir11, action_key="action_tcp") 

    # input_dir12 = "/media/Data/demoaccount2/robot_manipulation/paper_extension_data/wiping/mixed/train"   
    # output_dir12 = "/media/Data/demoaccount2/robot_manipulation/paper_extension_data/conversion_ready/without_proprio/wiping_mixed"    
    # process_episodes(input_dir12, output_dir12, action_key="action_tcp") 

    # PAPER EXTENSION DATASETS --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------











    # input_dir = "/media/Data/demoaccount2/robot_manipulation/lightbulb_VR/train"   # directory with episode_*.h5 and episode_*_images/
    # output_dir = "/media/Data/demoaccount2/rlds_dataset_builder/lightbulb_VR"      # directory to write zipped h5s compatible with RLDS
    # process_episodes(input_dir, output_dir, action_key="action_raw")
    # input_dir2 = "/home/demoaccount/Data/demoaccount2/rlds_dataset_builder/ur10e_dataset_autogen_random_fivehz_joint_space/data/train"   # directory with episode_*.h5 and episode_*_images/
    # output_dir2 = "/home/demoaccount/Data/demoaccount2/rlds_dataset_builder/ur10e_dataset_autogen_random_fivehz/dataZ_joint_space/train"      # directory to write zipped h5s compatible with RLDS

    # process_episodes(input_dir2, output_dir2)
