import numpy as np
import pickle

if __name__ == '__main__':
    pkl_path = 'Dataset/first_ground_truth_alice_no_sword1.pkl'
    with open(pkl_path, 'rb') as f:
        skeleton_dict = pickle.load(f)