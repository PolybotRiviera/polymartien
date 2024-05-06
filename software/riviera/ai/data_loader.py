import os
import sys
import torch
from torch._tensor import Tensor
from torch.utils.data import Dataset, DataLoader
import multiprocessing
import tqdm
import numpy as np
import imageio.v3


class PlantsDataset(Dataset):
    def __init__(self, csv_file: str, device: str = None) -> None:
        self.file = csv_file
        with open(csv_file, "r") as f:
            self._len = len(f.readlines()) - 1
        self.device = device
        self.images_folder = os.path.join(os.path.dirname(csv_file), "images")
        self.images_folder = csv_file.replace(".csv", "/")

    def __len__(self) -> int:
        return self._len

    def __getitem__(self, index: int) -> tuple[torch.Tensor, torch.Tensor]:
        with open(self.file, "r") as f:
            line = f.readlines()[index + 1]
        inputs = line.strip().split(",")
        imgs = torch.stack(
            [
                torch.Tensor(
                    imageio.v3.imread(
                        os.path.join(self.images_folder, img_path)
                    ).swapaxes(0, 2)
                ).to(device=self.device)
                for img_path in inputs[1:5]
            ]
        )
        inputs = [float(inp) for inp in inputs[5:]]
        inputs = torch.tensor(inputs, device=self.device).reshape((36 * 5, 2))
        # print(imgs.shape, inputs.shape)
        return imgs, inputs


def plants_dataloader(
    csv_file: str,
    batch_size: int = 64,
    num_worker: int = 0,
    prefetch: int = 0,
    shuffle: bool = False,
    device: str = None,
) -> DataLoader:
    return DataLoader(
        PlantsDataset(csv_file, device),
        batch_size,
        num_workers=num_worker,
        prefetch_factor=prefetch if num_worker > 0 else None,
        shuffle=shuffle,
        persistent_workers=True if num_worker else False,
    )
