import torch
from torch import nn
import torch.optim.lr_scheduler as lrs
from torch.utils.data import DataLoader
import matplotlib.pyplot as plt
import tqdm


def train_step(
    model: nn.Module,
    dataloader: DataLoader,
    loss_fn: nn.Module,
    optimizer: torch.optim.Optimizer,
    device: str = "cpu",
):
    model.train()
    av_loss, av_acc = 0, 0
    for images, solution in tqdm.tqdm(dataloader):
        # print(images.shape)
        image0 = images[0, 0].to(device).float().reshape([1, 3, 640, 480])
        image1 = images[0, 1].to(device).float().reshape([1, 3, 640, 480])
        image2 = images[0, 2].to(device).float().reshape([1, 3, 640, 480])
        image3 = images[0, 3].to(device).float().reshape([1, 3, 640, 480])
        prediction0 = model(image0)
        prediction1 = model(image1)
        prediction2 = model(image2)
        prediction3 = model(image3)
        loss = loss_fn(solution, prediction0, prediction1, prediction2, prediction3).to(
            device
        )

        acc = 1 - loss
        av_loss += loss.item()
        av_acc += acc.item()
        optimizer.zero_grad()
        # loss.zero_grad()
        # loss.requires_grad = True
        loss.backward()

        # X = X.to(device).float()
        # y_ = [CircImg.from_inputs_tens(yi) for yi in y]
        # preds = model(X)
        # for i in range(len(y_)): y_[i].manip_from_tens(preds[i])
        # loss = loss_fn(y_).to(device)
        # acc = 1 - loss
        # av_loss += loss.item()
        # av_acc += acc.item()
        # optimizer.zero_grad()
        # loss.backward()

        nn.utils.clip_grad_norm_(model.parameters(), max_norm=1.0)

        for param in model.parameters():
            if param.grad is not None:
                param.grad.data.masked_fill_(torch.isnan(param.grad.data), 0)

        # grad_norms = [p.grad.data.norm(2).item() for p in model.parameters() if p.grad is not None]
        # # scaling_factor = .01 / max(grad_norms)
        # # for param in model.parameters():
        # #     if param.grad is not None:
        # #         param.grad.data *= scaling_factor

        optimizer.step()
        # if hasattr(model, "apply_training_constrains"): model.apply_training_constrains()
        # del X, y, preds, loss, acc
        # if device=="cuda": torch.cuda.empty_cache()
    av_loss = av_loss / len(dataloader)
    av_acc = av_acc / len(dataloader)
    return av_loss, av_acc


def test_step(
    model: nn.Module, dataloader: DataLoader, loss_fn: nn.Module, device: str = "cpu"
):
    model.eval()
    av_loss, av_acc = 0, 0
    with torch.inference_mode():
        for images, solution in tqdm.tqdm(dataloader):
            image0 = images[0, 0].to(device).float().reshape([1, 3, 640, 480])
            image1 = images[0, 1].to(device).float().reshape([1, 3, 640, 480])
            image2 = images[0, 2].to(device).float().reshape([1, 3, 640, 480])
            image3 = images[0, 3].to(device).float().reshape([1, 3, 640, 480])
            prediction0 = model(image0)
            prediction1 = model(image1)
            prediction2 = model(image2)
            prediction3 = model(image3)
            loss = loss_fn(
                solution, prediction0, prediction1, prediction2, prediction3
            ).to(device)

            acc = 1 - loss
            av_loss += loss.item()
            av_acc += acc.item()
            # del X, y, preds, loss, acc
            # if device=="cuda": torch.cuda.empty_cache()
    av_loss = av_loss / len(dataloader)
    av_acc = av_acc / len(dataloader)
    return av_loss, av_acc


def train(
    model: nn.Module,
    dataloader_train: DataLoader,
    dataloader_test: DataLoader,
    loss_fn: nn.Module,
    optimizer: torch.optim.Optimizer,
    scheduler: lrs.StepLR,
    results: dict = {
        "train_loss": [],
        "train_acc": [],
        "test_loss": [],
        "test_acc": [],
        "move_final_acc": [],
        "move_names": ["dx", "dy", "dz", "rx", "ry", "rz", "overall"],
    },
    device: str = "cpu",
    epochs: int = 100,
):
    for epoch in range(epochs):
        print(f"Starting epoch {epoch+1}/{epochs}")
        train_loss, train_acc = train_step(
            model, dataloader_train, loss_fn, optimizer, device
        )
        test_loss, test_acc = test_step(model, dataloader_test, loss_fn, device)
        if epoch > -5:
            print(
                f"Epoch: {epoch+1}/{epochs} | "
                f"train_loss: {train_loss:.6f} | "
                f"train_acc: {train_acc:.6f} | "
                f"test_loss: {test_loss:.6f} | "
                f"test_acc: {test_acc:.6f} | "
                f"lr: {scheduler.get_last_lr()[0]:.6f}"
            )
            results["train_loss"].append(train_loss)
            results["train_acc"].append(train_acc)
            results["test_loss"].append(test_loss)
            results["test_acc"].append(test_acc)
            scheduler.step()

    results["cat"] = ["dx", "dy", "dz", "rx", "ry", "rz", "overall"]
    # print("Movement evaluation:")
    # results["move_final_acc"] = []
    # for weight in torch.cat((torch.eye(6), torch.Tensor([[1 for _ in range(6)]]))).to(
    #     device=device
    # ):
    #     _, test_acc = test_step(model, dataloader_test, loss_fn, device)

    #     results["move_final_acc"].append(test_acc)
    print(results["move_final_acc"])

    return results


def plot_results(
    *results: dict[str, list[float]],
    names: list[str],
    times: list[float],
    move_names: list[str] = ["dx", "dy", "dz", "rx", "ry", "rz", "overall"],
    legend: bool = True,
):
    losses = [result["train_loss"] for result in results]
    test_losses = [result["test_loss"] for result in results]
    accs = [result["train_acc"] for result in results]
    test_accs = [result["test_acc"] for result in results]
    move_final_acc = [result["move_final_acc"] for result in results]

    # print(losses, test_losses, losses+test_losses, sep="\n")
    # print(np.array(losses + test_losses))

    plt.figure(figsize=(15, 7))

    plt.subplot(2, 2, 1)
    for n, loss in enumerate(losses):
        plt.plot(range(len(loss)), loss, "-.", label=f"train_loss {names[n]}")
    for n, test_loss in enumerate(test_losses):
        plt.plot(range(len(test_loss)), test_loss, label=f"test_loss {names[n]}")

    plt.title("Loss")
    plt.xlabel("Epochs")
    if legend:
        plt.legend()
    # plt.ylim(0, 0.25)

    # Plot accuracy
    plt.subplot(2, 2, 2)
    for n, acc in enumerate(accs):
        plt.plot(range(len(acc)), acc, "-.", label=f"train_accuracy {names[n]}")
    for n, test_acc in enumerate(test_accs):
        plt.plot(range(len(test_acc)), test_acc, label=f"test_accuracy {names[n]}")
    plt.title("Accuracy")
    plt.xlabel("Epochs")
    if legend:
        plt.legend()
    # plt.ylim(0.75, 1)
    times_ = [
        [times[i + j] for i in range(len(times) // len(names))]
        for j in range(len(names))
    ]

    # Plot time
    plt.subplot(2, 2, 3)
    plt.plot(names, times_, "x")
    plt.title("Time for each variation")
    plt.xlabel("Models")
    # plt.legend()

    # Plot acc mov
    plt.subplot(2, 2, 4)
    # for n, move_final_acc_n in enumerate(move_final_acc):
    #     plt.plot(move_names, move_final_acc_n, label=names[n])  # TODO
    # plt.title("Accuracy by movement")
    # plt.xlabel("Movement")
    # plt.ylim(0.6, 1)

    if legend:
        plt.legend()

    acclastmeans = [
        sum(acclist[len(acclist) // 2 :]) / len(acclist) // 2 for acclist in test_accs
    ]
    accmaxs = [max(acclist) for acclist in test_accs]
    listtimes = [
        (name_, time_, acclastmean_, accmax_)
        for name_, time_, acclastmean_, accmax_ in zip(
            names, times_, acclastmeans, accmaxs
        )
    ]
    print("(name, time, acclastmean, accmax)")

    listtimes.sort(key=lambda x: sum(x[1]))
    print("Ordered by mean times:")
    print(listtimes)
    print(
        "max delta:",
        abs(sum(listtimes[0][1]) - sum(listtimes[-1][1])) / sum(listtimes[-1][1]) * 100,
        "%",
    )

    listtimes.sort(key=lambda x: -x[3])
    print("\nOrdered by max accuracy:")
    print(listtimes)

    print("max test accuracy :", max([max(acclist) for acclist in test_accs]))
