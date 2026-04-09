#!/usr/bin/env python3
"""Visualize recorded joint data curves from CSV files.

Usage:
    python visualize_recording.py                          # Use file dialog
    python visualize_recording.py recording_20240101.csv   # Specify file
"""
from __future__ import annotations

import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

JOINT_NAMES = [
    'LF_HipA', 'LR_HipA', 'RF_HipA', 'RR_HipA',
    'LF_HipF', 'LR_HipF', 'RF_HipF', 'RR_HipF',
    'LF_Knee', 'LR_Knee', 'RF_Knee', 'RR_Knee',
]


def load_data(csv_path: str) -> pd.DataFrame:
    return pd.read_csv(csv_path)


def plot_all_joints(df: pd.DataFrame, title: str = '') -> None:
    t = df['time_s'].values
    fig, axes = plt.subplots(4, 3, figsize=(18, 14), sharex=True)
    fig.suptitle(title or 'Joint Position Recording', fontsize=16)
    axes_flat = axes.flatten()

    for i, name in enumerate(JOINT_NAMES):
        ax = axes_flat[i]
        target_col = f'target_{name}'
        robot_col = f'robot_{name}'

        if target_col in df.columns:
            ax.plot(t, df[target_col], label='target', color='#00e5ff', linewidth=1.2)
        if robot_col in df.columns:
            ax.plot(t, df[robot_col], label='robot', color='#ffd740', linewidth=1.2)

        ax.set_title(name, fontsize=11)
        ax.legend(loc='upper right', fontsize=8)
        ax.grid(True, alpha=0.3)
        ax.set_ylabel('rad')

    for ax in axes_flat[9:]:
        ax.set_xlabel('time (s)')

    plt.tight_layout()


def plot_single_joint(df: pd.DataFrame, joint_name: str) -> None:
    t = df['time_s'].values
    fig, axes = plt.subplots(2, 1, figsize=(14, 8), sharex=True,
                              gridspec_kw={'height_ratios': [3, 1]})
    fig.suptitle(f'Joint: {joint_name}', fontsize=16)

    target_col = f'target_{joint_name}'
    robot_col = f'robot_{joint_name}'

    ax = axes[0]
    if target_col in df.columns:
        ax.plot(t, df[target_col], label='target', color='#00e5ff', linewidth=1.5)
    if robot_col in df.columns:
        ax.plot(t, df[robot_col], label='robot', color='#ffd740', linewidth=1.5)
    ax.set_ylabel('position (rad)')
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.3)

    # Error plot
    ax_err = axes[1]
    if target_col in df.columns and robot_col in df.columns:
        error = df[robot_col].values - df[target_col].values
        ax_err.plot(t, error, label='error', color='#ff5252', linewidth=1.0)
        ax_err.axhline(y=0, color='gray', linestyle='--', alpha=0.5)
    ax_err.set_ylabel('error (rad)')
    ax_err.set_xlabel('time (s)')
    ax_err.legend(loc='upper right')
    ax_err.grid(True, alpha=0.3)

    plt.tight_layout()


def main() -> None:
    if len(sys.argv) > 1:
        csv_path = sys.argv[1]
    else:
        from tkinter import Tk, filedialog
        root = Tk()
        root.withdraw()
        csv_path = filedialog.askopenfilename(
            title='Select recording CSV',
            filetypes=[('CSV files', '*.csv'), ('All files', '*.*')],
            initialdir=str(Path(__file__).resolve().parent),
        )
        if not csv_path:
            print('No file selected.')
            return

    df = load_data(csv_path)
    print(f'Loaded {len(df)} samples, duration: {df["time_s"].iloc[-1]:.2f}s')

    plot_all_joints(df, title=Path(csv_path).name)
    plt.show()


if __name__ == '__main__':
    main()
