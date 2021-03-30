import numpy as np
import sys


def get_sync_poses(gt_poses, gt_stamps, slam_poses, slam_stamps):
    j = 0
    gt_poses_sync = np.zeros_like(slam_poses)
    for i in range(len(slam_stamps)):
        while j < len(gt_stamps) and gt_stamps[j] < slam_stamps[i]:
            j += 1
        if j == 0:
            gt_poses_sync[i] = gt_poses[0]
        elif j == len(gt_stamps):
            gt_poses_sync[i] = gt_poses[-1]
        else:
            alpha = (slam_stamps[i] - gt_stamps[j - 1]) / (gt_stamps[j] - gt_stamps[j - 1])
            gt_poses_sync[i] = alpha * gt_poses[j] + (1 - alpha) * gt_poses[j - 1]
    return gt_poses_sync


def get_ate(slam_poses, gt_poses):
    assert len(slam_poses) == len(gt_poses)
    return np.sqrt(np.mean(np.sum((slam_poses[:, :3] - gt_poses[:, :3]) ** 2, axis=1)))


def get_rpe(slam_poses, gt_poses, correction_angle=0):
    assert len(slam_poses) == len(gt_poses)
    gt_shifts = gt_poses[1:, :3] - gt_poses[:-1, :3]
    gt_shifts_new = gt_shifts.copy()
    gt_shifts_new[:, 0] = gt_shifts[:, 0] * np.cos(gt_poses[:-1, -1] + np.pi + correction_angle) + \
                          gt_shifts[:, 1] * np.sin(gt_poses[:-1, -1] + np.pi + correction_angle)
    gt_shifts_new[:, 1] = -gt_shifts[:, 0] * np.sin(gt_poses[:-1, -1] + np.pi + correction_angle) + \
                          gt_shifts[:, 1] * np.cos(gt_poses[:-1, -1] + np.pi + correction_angle)
    slam_shifts = slam_poses[1:, :3] - slam_poses[:-1, :3]
    slam_shifts_new = slam_shifts.copy()
    slam_shifts_new[:, 0] = slam_shifts[:, 0] * np.cos(slam_poses[:-1, -1]) + \
                          slam_shifts[:, 1] * np.sin(slam_poses[:-1, -1])
    slam_shifts_new[:, 1] = -slam_shifts[:, 0] * np.sin(slam_poses[:-1, -1]) + \
                          slam_shifts[:, 1] * np.cos(slam_poses[:-1, -1])
    return np.sqrt(np.mean(np.sum((slam_shifts_new - gt_shifts_new) ** 2, axis=1)))


if __name__ == '__main__':
    gt_poses_file = sys.argv[1]
    slam_poses_file = sys.argv[2]
    gt_stamps_file = gt_poses_file.replace('.txt', '_timestamps.txt')
    slam_stamps_file = slam_poses_file.replace('.txt', '_timestamps.txt')
    gt_poses = np.loadtxt(gt_poses_file)
    gt_stamps = np.loadtxt(gt_stamps_file)
    slam_poses = np.loadtxt(slam_poses_file)
    slam_stamps = np.loadtxt(slam_stamps_file)
    gt_poses_sync = get_sync_poses(gt_poses, gt_stamps, slam_poses, slam_stamps)
    if sys.argv[3] not in ['ate', 'rpe']:
        print('Incorrect arg: {}! The third argument must be "ate" or "rpe"'.format(sys.argv[3]))
        assert(False)
    if len(sys.argv) > 4:
        correction_angle = float(sys.argv[4])
    else:
        correction_angle = 0
    if sys.argv[3] == 'ate':
        print('ATE value: {}'.format(get_ate(slam_poses, gt_poses_sync)))
    else:
        print('RPE value: {}'.format(get_rpe(slam_poses, gt_poses_sync, correction_angle)))