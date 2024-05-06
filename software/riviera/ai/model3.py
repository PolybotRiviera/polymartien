import torch

# import numpy as np

from modules import Centroids2d, CentroidsCrop2d


def normal_init(layer):
    if isinstance(layer, torch.nn.Linear):
        torch.nn.init.xavier_normal_(layer.weight)


class PlantNet(torch.nn.Module):
    def __init__(self, ACTIVATION=torch.nn.RReLU) -> None:
        super().__init__()
        self._device = "cuda" if torch.cuda.is_available() else "cpu"

        self.conv_in = torch.nn.Sequential(
            torch.nn.Conv2d(3, 9, kernel_size=15, padding=7, device=self._device),
            ACTIVATION(),
            torch.nn.Conv2d(9, 18, kernel_size=5, padding=2, device=self._device),
            torch.nn.MaxPool2d(kernel_size=2),
            torch.nn.Conv2d(18, 36, kernel_size=3, padding=1, device=self._device),
        )
        self.conv_in.apply(normal_init)
        self.loc1 = torch.nn.Sequential(
            ACTIVATION(),
            Centroids2d(36, sigma=3, device=self._device),
            ACTIVATION(),
        )
        self.crop = CentroidsCrop2d(36, (100, 100), device=self._device)
        self.conv_deep = torch.nn.Sequential(
            ACTIVATION(),
            torch.nn.Conv2d(36, 36, kernel_size=7, device=self._device),
            ACTIVATION(),
        )
        self.conv_out = torch.nn.Sequential(
            torch.nn.AdaptiveMaxPool2d((15, 15)),
            torch.nn.Flatten(),
            torch.nn.Linear(15 * 15 * 36, 25, device=self._device),
        )
        self.conv_out.apply(normal_init)
        self.reduce = torch.nn.Sequential(
            ACTIVATION(),
            torch.nn.Flatten(),
            torch.nn.Linear(36 * 5, 100, device=self._device),
        )
        self.out = torch.nn.Sequential(
            ACTIVATION(),
            torch.nn.Linear(100 + 25, 100, device=self._device),
            ACTIVATION(),
            torch.nn.Linear(100, 72, device=self._device),
        )
        self.out.apply(normal_init)

        self.to(self._device)

    def forward(self, x):
        # print(f"size: {x.size(0)}:{x.size(1)}:{x.size(2)}:{x.size(3)}")
        # print(x.shape)

        conv_in = self.conv_in(x)
        loc1 = self.loc1(conv_in)
        conv = self.conv_deep(self.crop(conv_in, loc1)) # .mean(dim=1)[..., :36].unsqueeze(1)
        return self.out(
            torch.cat([self.reduce(loc1), self.conv_out(conv)], dim=1)
        ).reshape((36 * 1, 2))




class PlantNetNoCrop(torch.nn.Module):
    def __init__(self, ACTIVATION=torch.nn.RReLU) -> None:
        super().__init__()
        self._device = "cuda" if torch.cuda.is_available() else "cpu"

        self.conv_in = torch.nn.Sequential(
            torch.nn.Conv2d(3, 9, kernel_size=15, padding=7, device=self._device),
            ACTIVATION(),
            torch.nn.Conv2d(9, 18, kernel_size=5, padding=2, device=self._device),
            torch.nn.MaxPool2d(kernel_size=2),
            torch.nn.Conv2d(18, 36, kernel_size=3, padding=1, device=self._device),
            ACTIVATION(),
            torch.nn.Conv2d(36, 36, kernel_size=3, padding=1, device=self._device),
        )
        self.conv_in.apply(normal_init)
        self.conv_deep = torch.nn.Sequential(
            ACTIVATION(),
            torch.nn.Conv2d(36, 36, kernel_size=7, device=self._device),
            ACTIVATION(),
        )
        self.conv_out = torch.nn.Sequential(
            torch.nn.AdaptiveMaxPool2d((15, 15)),
            torch.nn.Flatten(),
            torch.nn.Linear(15 * 15 * 36, 125, device=self._device),
        )
        self.conv_out.apply(normal_init)
        self.reduce = torch.nn.Sequential(
            ACTIVATION(),
            torch.nn.Flatten(),
            torch.nn.Linear(36 * 5, 100, device=self._device),
        )
        self.out = torch.nn.Sequential(
            ACTIVATION(),
            torch.nn.Linear(125, 100, device=self._device),
            ACTIVATION(),
            torch.nn.Linear(100, 72, device=self._device),
        )
        self.out.apply(normal_init)

        self.to(self._device)

    def forward(self, x):
        # print(f"size: {x.size(0)}:{x.size(1)}:{x.size(2)}:{x.size(3)}")
        # print(x.shape)

        conv_in = self.conv_in(x)
        conv = self.conv_deep(conv_in)
        return self.out(self.conv_out(conv)).reshape((36 * 1, 2))
