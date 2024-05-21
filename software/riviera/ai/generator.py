import numpy as np
import torch

ANGLE_RANGE = torch.pi / 3


class PlantLoss(torch.nn.Module):
    def __init__(self) -> None:
        super().__init__()

        self.loss = torch.nn.MSELoss()

    def forward(
        self,
        y: torch.Tensor,
        preds0: torch.Tensor,
        preds1: torch.Tensor,
        preds2: torch.Tensor,
        preds3: torch.Tensor,
    ) -> torch.Tensor:
        # print(len(y.shape), y.size())
        # assert len(y.shape) == 2 and y.size(1) == 13
        # assert len(preds.shape) == 2 and preds.size(1) == 6
        # print(y)
        # print("this tensor is on ",y.device)
        # print("##########################################################################")
        # print(preds0.size())

        # sort y=(r, theta) by theta
        # _, indices = torch.sort(y[:, 1])
        # y = y[:, indices]
        res = 0
        for preds, i in zip([preds0, preds1, preds2, preds3], range(4)):
            # print(preds.shape)
            # factor = torch.where(
            #     torch.abs(preds[:, 1]) < ANGLE_RANGE / 2,
            #     torch.ones_like(preds[:, 1]),
            #     torch.zeros_like(preds[:, 1]),
            # )
            # preds = torch.stack((preds[:, 0] * factor, preds[:, 1]*factor)).T

            # _, indices = torch.sort(input=preds[:, 1])
            # preds = preds[indices]

            factor = torch.where(
                torch.abs((y[0, :36, 1] - torch.pi / 2 * i)%(torch.pi*2)) < ANGLE_RANGE / 2,
                torch.ones_like(y[0, :36, 1]),
                torch.zeros_like(y[0, :36, 1]),
            )
            y_ = torch.stack((y[0, :36, 0] * factor, y[0, :36, 1] * factor)).T      # y_=y[0, :36, :].T

            _, indices = torch.sort(y_[:, 1])
            y_: torch.Tensor = y_[indices][:36, :]
            # print(y_.shape, preds.shape)
            res += self.loss((y_%6.28)/6.28, preds)
        return res / 4
