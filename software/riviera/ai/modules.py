import torch
import torch.nn as nn
import torch.nn.functional as F
import numpy as np

# source for the centroid extraction and cropping modules:
# https://kornia.readthedocs.io/en/latest/_modules/kornia/contrib/connected_components.html
# https://github.com/zsef123/Connected_components_PyTorch


class Normalize2d(nn.Module):
    def __init__(self, device: str = None) -> None:
        """Normalize all elements from 2d feature maps to values between 0 and 1"""
        super().__init__()
        self._device = device if device else "cpu"
        self.to(self._device)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        view_size = x.shape[:-2] + torch.Size([1, 1])
        min_values = (
            x.min(dim=-1).values.min(dim=-1).values.view(view_size).expand(x.shape)
        )
        max_values = (
            x.max(dim=-1).values.max(dim=-1).values.view(view_size).expand(x.shape)
        )
        return torch.where(
            (max_values - min_values) != 0,
            (x - min_values) / (max_values - min_values),
            0,
        )


class Centroids2d(nn.Module):
    def __init__(
        self,
        channels: int,
        sigma: float = 2.0,
        learnable_thr: bool = True,
        device: str = None,
    ) -> None:
        """Extract centroid positions as well as angle and diameters from 2d feature maps"""
        super().__init__()
        self._device = device if device else "cpu"
        # print(f"my device centroid: {self._device}")

        self.channels = channels
        self.sigma = torch.tensor(sigma).to(self._device)
        if learnable_thr:
            self.thr = nn.Parameter(
                torch.ones(self.channels, 1, 1) * -10, requires_grad=True
            )
        else:
            self.thr = torch.ones(self.channels, 1, 1).to(self._device) * -10
        self.normalize = Normalize2d(device=self._device)

        # constants
        self.SQRT2 = torch.sqrt(torch.tensor(2)).to(self._device)
        self.PI = torch.tensor(np.pi).to(self._device)

        # for p in self.parameters(): p.register_hook(lambda grad: 0 if torch.isnan(grad) else grad)

        self.to(self._device)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        assert len(x.shape) == 4 and x.size(1) == self.channels
        # print(f"my device forward centroid: {self._device}")

        b = x.size(0)
        width = x.size(-1)
        height = x.size(-2)

        # squash batch and channel dimension together
        x_flat = x.view(b * self.channels, height, width)

        # normalize and inverse if the mean > .5
        x_flat = self.normalize(x_flat)
        x_flat = torch.where(
            x_flat.mean(dim=(1, 2), keepdim=True) < 0.5,
            x_flat,
            torch.ones_like(x_flat) - x_flat,
        )

        # clamp at thr (with thr being the sigmoid of learnable self.thr) using relu, then normalizing again
        thr = self.thr.sigmoid().repeat(b, 1, 1)
        x_flat = torch.relu(x_flat - thr) + thr
        x_flat = self.normalize(x_flat) * 100

        # calculate centroids using sum(coords*values)/sum_values
        x_coords, y_coords = torch.meshgrid(
            torch.arange(width, device=self._device),
            torch.arange(height, device=self._device),
            indexing="xy",
        )
        x_coords = x_coords.expand(b * self.channels, -1, -1)
        y_coords = y_coords.expand(b * self.channels, -1, -1)

        sum_values = x_flat.sum((-1, -2))
        c_x = torch.where(
            sum_values != 0, (x_flat * x_coords).sum((-1, -2)) / sum_values, width / 2
        )  # / width
        c_y = torch.where(
            sum_values != 0, (x_flat * y_coords).sum((-1, -2)) / sum_values, height / 2
        )  # / height

        sm_x = torch.where(
            sum_values != 0,
            (
                x_flat
                * torch.pow(x_coords - c_x[:, None, None].repeat(1, height, width), 2)
            ).sum((-1, -2))
            / sum_values,
            0,
        )
        sm_y = torch.where(
            sum_values != 0,
            (
                x_flat
                * torch.pow(y_coords - c_y[:, None, None].repeat(1, height, width), 2)
            ).sum((-1, -2))
            / sum_values,
            0,
        )
        sm_xy = torch.where(
            sum_values != 0,
            (
                x_flat
                * (x_coords - c_x[:, None, None].repeat(1, height, width))
                * (y_coords - c_y[:, None, None].repeat(1, height, width))
            ).sum((-1, -2))
            / sum_values,
            0,
        )

        # precalculating the inputs of the sqrt's so we can zero out negative values, thus avoiding nan's
        # the zero'ed values are not supposed to be used anyway beacuse of the where's but would still produce nan-grads if not zero'ed
        sqrt_1 = torch.pow(sm_x - sm_y, 2) + 4 * torch.pow(sm_xy, 2)
        sqrt_1 = torch.where(sqrt_1 < 0, 0, sqrt_1)
        sqrt_x2 = sm_x + sm_y + torch.sign(sm_x - sm_y) * torch.sqrt(sqrt_1)
        sqrt_x2 = torch.where(sqrt_x2 < 0, 0, sqrt_x2)
        sqrt_x3 = (
            torch.pow(sm_x, 2) + torch.pow(sm_y, 2) + 2 * torch.abs(torch.pow(sm_xy, 2))
        )
        sqrt_x3 = torch.where(sqrt_x3 < 0, 0, sqrt_x3)
        sqrt_y2 = sm_x + sm_y - torch.sign(sm_x - sm_y) * torch.sqrt(sqrt_1)
        sqrt_y2 = torch.where(sqrt_y2 < 0, 0, sqrt_y2)
        sqrt_y3 = (
            torch.pow(sm_x, 2) + torch.pow(sm_y, 2) - 2 * torch.abs(torch.pow(sm_xy, 2))
        )
        sqrt_y3 = torch.where(sqrt_y3 < 0, 0, sqrt_y3)

        az = torch.where(
            sm_x != sm_y,
            torch.atan((2 * sm_xy) / (sm_x - sm_y)) * 0.5,
            torch.sign(sm_xy) * self.PI,
        )
        d_az_x = torch.where(
            sm_x != sm_y,
            self.sigma * self.SQRT2 * torch.sqrt(sqrt_x2),
            self.sigma * self.SQRT2 * torch.sqrt(sqrt_x3),
        )
        d_az_y = torch.where(
            sm_x != sm_y,
            self.sigma * self.SQRT2 * torch.sqrt(sqrt_y2),
            self.sigma * self.SQRT2 * torch.sqrt(sqrt_y3),
        )

        return torch.stack(
            [
                c_x[:, None, None] / width,
                c_y[:, None, None] / height,
                az[:, None, None],
                d_az_x[:, None, None] / width,
                d_az_y[:, None, None] / height,
            ],
            dim=1,
        ).view(x.shape[:-2] + torch.Size([5]))


class CentroidsCrop2d(nn.Module):
    def __init__(
        self, channels: int, out_size: tuple[int, int], device: str = None
    ) -> None:
        """crop 2d feature map given centroid positions as well as angle and diameters"""
        super().__init__()
        self._device = device if device else "cpu"
        # print(f"my device crop: {self._device}")

        self.channels = channels
        self.out_size = out_size
        self._theta = torch.zeros(self.channels, 2, 3, device=self._device)

        self.PI = torch.tensor(np.pi).to(self._device)

        self.to(device)

    def forward(self, x: torch.Tensor, centroids: torch.Tensor) -> torch.Tensor:
        # print(x.shape, self.channels)
        # print(centroids.shape)
        assert len(x.shape) == 4
        assert x.size(1) == self.channels
        assert (
            len(centroids.shape) == 3
            and centroids.size(2) == 5
            and centroids.size(1) == self.channels
        )
        assert x.size(0) == centroids.size(0)

        b = x.size(0)
        width = x.size(-1)
        height = x.size(-2)

        # squash batch and channel dimension together
        x_flat = x.view(b * self.channels, height, width)

        alpha = -centroids[..., 2].view(b * self.channels)
        center = 2 * centroids[..., 0:2].view(b * self.channels, 2) - 1
        regions = centroids[..., 3:5].view(b * self.channels, 2)

        # padding feature map to produce square (the rotation on non-square feature maps cannot be calculated correctly)
        pad_h = max(0, width - height)
        pad_w = max(0, height - width)
        pad_left = pad_w // 2
        pad_right = pad_w - pad_left
        pad_top = pad_h // 2
        pad_bottom = pad_h - pad_top
        x_flat = F.pad(
            x_flat, (pad_left, pad_right, pad_top, pad_bottom), mode="constant", value=0
        )
        center = center.clone()
        regions = regions.clone()
        center[:, 0] *= width / x_flat.size(-1)
        center[:, 1] *= height / x_flat.size(-2)
        regions[:, 0] *= width / x_flat.size(-1)
        regions[:, 1] *= height / x_flat.size(-2)

        # creating theta for affine_grid given rotation, diameters (transformed by given rotation) and center
        theta = self._theta.repeat(b, 1, 1)
        theta[:, 0, 0] = torch.cos(alpha) * regions[:, 0]
        theta[:, 0, 1] = torch.sin(alpha) * regions[:, 1]
        theta[:, 1, 0] = -torch.sin(alpha) * regions[:, 0]
        theta[:, 1, 1] = torch.cos(alpha) * regions[:, 1]
        theta[:, :, 2] = center

        # transforming feature map by creating affine grid with theta
        grid = F.affine_grid(
            theta,
            torch.Size([b * self.channels, 1]) + torch.Size(self.out_size),
            align_corners=True,
        )
        out = F.grid_sample(
            x_flat.unsqueeze(1),
            grid,
            mode="bilinear",
            padding_mode="zeros",
            align_corners=True,
        ).squeeze(1)
        return out.view(x.shape[:-2] + out.shape[-2:])
