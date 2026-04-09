import h5py

filenames = ["/media/Data/demoaccount2/robot_manipulation/data_with_fts_2/train/episode_00005.h5"]


# Put the two keys you want to print here.
# If left empty, the script will print the first two keys found.
keys_to_print = ["robot_state", "action", "action_tcp"]  # e.g. ["observations", "actions"]

def _sample_dataset(ds, max_items=10):
    try:
        if ds.ndim == 0:
            return ds[()]
        # build slices for each dimension limited by max_items
        slices = tuple(slice(0, min(dim, max_items)) for dim in ds.shape)
        return ds[slices]
    except Exception:
        # fallback: try to read the whole dataset (may be large)
        return ds[()]

for filename in filenames:
    try:
        with h5py.File(filename, 'r') as f:
            print(f"File: {filename}")
            all_keys = list(f.keys())
            print("Top-level keys:", all_keys)

            if not all_keys:
                print("No keys found in the file.")
                continue

            if not keys_to_print:
                # choose first two keys if user didn't specify
                keys = all_keys[:2]
            else:
                keys = keys_to_print

            for key in keys:
                print(f"\n--- Key: {key} ---")
                if key not in f:
                    print(f"(missing) {key} not found in file")
                    continue

                obj = f[key]
                # dataset
                if isinstance(obj, h5py.Dataset):
                    print("Type: Dataset")
                    print("Shape:", obj.shape, " dtype:", obj.dtype)
                    try:
                        sample = _sample_dataset(obj)
                        print("Sample data:", sample)
                    except Exception as e:
                        print("Could not read dataset:", e)
                # group
                elif isinstance(obj, h5py.Group):
                    print("Type: Group")
                    subkeys = list(obj.keys())
                    print("Contains subkeys:", subkeys)
                    for sub in subkeys:
                        subobj = obj[sub]
                        if isinstance(subobj, h5py.Dataset):
                            print(f"  Dataset '{sub}': shape={subobj.shape} dtype={subobj.dtype}")
                            try:
                                sample = _sample_dataset(subobj)
                                print(f"    Sample: {sample}")
                            except Exception as e:
                                print(f"    Could not read '{sub}': {e}")
                        else:
                            print(f"  Subgroup '{sub}' (skipping deeper inspection)")
                else:
                    print(f"Unknown object type for key '{key}': {type(obj)}")

            print("\n" + "-" * 40)
    except Exception as e:
        print(f"Could not open {filename}: {e}")
