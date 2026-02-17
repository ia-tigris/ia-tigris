import argparse
import os
import torch
from torch import nn
import torch.nn.functional as F
from torchvision import transforms
from torchvision.datasets import MNIST
from torch.utils.data import DataLoader, random_split
import pytorch_lightning as pl
from pytorch_lightning.callbacks import ModelCheckpoint
from pytorch_lightning.loggers import TensorBoardLogger


from dataloading import MctsStateValueDataset
from networks import LinearModel
from torch.utils.data import Subset
from sklearn.model_selection import train_test_split

def train_val_dataset(dataset, val_split=0.2):
    train_idx, val_idx = train_test_split(
        list(range(len(dataset))), test_size=val_split
    )
    datasets = {}
    datasets["train"] = Subset(dataset, train_idx)
    datasets["val"] = Subset(dataset, val_idx)
    return datasets

class ValueEstimator(pl.LightningModule):
    def __init__(self, model):
        super().__init__()
        self.model = model

    def training_step(self, batch, batch_idx):
        # training_step defines the train loop.
        drone_state, particle_state, value = batch
        value_hat = self.model(drone_state, particle_state)
        loss = F.mse_loss(value_hat, value)
        self.log(
            "train_loss", loss, on_step=True, on_epoch=True, prog_bar=True, logger=True
        )
        return loss

    def validation_step(self, batch, batch_idx):
        drone_state, particle_state, value = batch
        value_hat = self.model(drone_state, particle_state)
        loss = F.mse_loss(value_hat, value)
        self.log(
            "val_loss", loss, on_step=True, on_epoch=True, prog_bar=True, logger=True
        )
        return loss

    def configure_optimizers(self):
        optimizer = torch.optim.Adam(self.parameters(), lr=1e-3)
        return optimizer


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="MCTS State Value Prediction")
    parser.add_argument("--name", type=str, nargs="?", default=None)
    parser.add_argument("--data_dir", type=str, default="train_data")
    parser.add_argument("--batch_size", type=int, default=128)
    parser.add_argument("--num_workers", type=int, default=8)
    parser.add_argument("--from_checkpoint", type=str, nargs="?", default=None)
    args = parser.parse_args()

    dataset = MctsStateValueDataset(args.data_dir)
    datasets = train_val_dataset(dataset)
    dataloaders = {
        x: DataLoader(
            datasets[x],
            args.batch_size,
            shuffle=True if x == "train" else False,
            num_workers=args.num_workers,
        )
        for x in ["train", "val"]
    }

    # model
    value_estimator = ValueEstimator(LinearModel())
    # train model
    checkpoint_callback = ModelCheckpoint(
        save_top_k=5, monitor="val_loss", filename="best-val-{epoch:02d}-{val_loss:.4E}"
    )

    logger = TensorBoardLogger("lightning_logs", name=args.name)

    trainer = pl.Trainer(
        resume_from_checkpoint=args.from_checkpoint,
        accelerator="gpu",
        devices=1,
        logger=logger,
        log_every_n_steps=10,
        callbacks=[checkpoint_callback],
        max_epochs=-1
    )
    trainer.fit(
        model=value_estimator,
        train_dataloaders=dataloaders["train"],
        val_dataloaders=dataloaders["val"],
    )
