import h5py
import sys

# def print_structure(name, obj):
    # if isinstance(obj, h5py.Group):
        # print(f"[Group] {name}")
    # elif isinstance(obj, h5py.Dataset):
        # print(f"[Dataset] {name} | shape: {obj.shape} | dtype: {obj.dtype}")
        
def print_structure(name, obj):
    if isinstance(obj, h5py.Group):
        print(f"[Group] {name}")
    elif isinstance(obj, h5py.Dataset):
        if name == "episode_info":
            data = obj[()]
            lang_instruction = data["language_instruction"][0].decode("utf-8")
            is_success = data["is_success"][0]
            episode_id = data["episode_id"][0]
            print(f"[Dataset] {name} | language_instruction: \"{lang_instruction}\" | is_success: {is_success} | episode_id: {episode_id}")
        # elif name == "fts_info":
        #     data = obj[()]
        #     print(f"[Dataset] {name} | contents:")
        #     print(data)
        else:
            print(f"[Dataset] {name} | shape: {obj.shape} | dtype: {obj.dtype}")


def inspect_h5_structure(file_path):
    with h5py.File(file_path, 'r') as f:
        f.visititems(print_structure)

if __name__ == "__main__":

    inspect_h5_structure("/media/Data/demoaccount2/robot_manipulation/data_with_fts_2/train/episode_00005.h5")

    #inspect_h5_structure("/home/demoaccount/Data/demoaccount2/robot_manipulation/data_zippered/train/episode_00033.h5")
    #inspect_h5_structure("/home/demoaccount/Data/demoaccount2/rlds_dataset_builder/ur10e_dataset_stuffy_rs_mix/data_stuffy_rs_mix/train/episode_000002.h5")
    #inspect_h5_structure("/home/demoaccount/Data/demoaccount2/rlds_dataset_builder/mega_stuffy_rs_mix/success_episodes/episode_000096.h5")
   
