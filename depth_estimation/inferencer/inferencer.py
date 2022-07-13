# Copyright (c) OpenMMLab. All rights reserved.
import cv2
import mmcv
import numpy as np
import torch
from mmcv.image import tensor2imgs
from mmcv.parallel import MMDataParallel
from mmcv.runner import get_dist_info, init_dist, load_checkpoint, wrap_fp16_model
from mmcv.utils import DictAction

from depth_estimation.datasets import build_dataloader, build_dataset
from depth_estimation.models import build_depther


class Inferencer:
    def __init__(self, args, dataload_path) -> None:
        cfg = mmcv.Config.fromfile(args.model_config)
        # set cudnn_benchmark
        if cfg.get("cudnn_benchmark", False):
            torch.backends.cudnn.benchmark = True

        cfg.model.pretrained = None
        cfg.data.test.test_mode = True
        if args.dataset == "ace":
            cfg.data.test.data_root = dataload_path
        elif args.dataset == "kitti":
            cfg.data.test.data_root = "dataset/kitti"

        # build the dataloader
        dataset = build_dataset(cfg.data.test)
        self.data_loader = build_dataloader(
            dataset,
            samples_per_gpu=1,
            workers_per_gpu=cfg.data.workers_per_gpu,
            dist=False,
            shuffle=False,
        )

        # build the model and load checkpoint
        cfg.model.train_cfg = None
        self.model = build_depther(cfg.model, test_cfg=cfg.get("test_cfg"))

        self.device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
        print(self.device)

        fp16_cfg = cfg.get("fp16", None)
        if fp16_cfg is not None:
            wrap_fp16_model(self.model)

        # for other models
        self.checkpoint = load_checkpoint(self.model, args.checkpoint, map_location="cpu")

        # clean gpu memory when starting a new evaluation.
        torch.cuda.empty_cache()
        self.model = MMDataParallel(self.model, device_ids=[0])
        self.model.eval()

    def infer(self, depth_path):
        result_depths = []
        self.model.eval()
        dataset = self.data_loader.dataset
        prog_bar = mmcv.ProgressBar(len(dataset))
        # The pipeline about how the data_loader retrieval samples from dataset:
        # sampler -> batch_sampler -> indices
        # The indices are passed to dataset_fetcher to get data from dataset.
        # data_fetcher -> collate_fn(dataset[index]) -> data_sample
        # we use batch_sampler to get correct data idx
        loader_indices = self.data_loader.batch_sampler

        for batch_indices, data in zip(loader_indices, self.data_loader):
            with torch.no_grad():
                result_depth = self.model(return_loss=False, **data)
            print(result_depth[0].shape)
            result_depths.append(result_depth[0])

            # add save depth_image
            img_tensor = data["img"][0]
            img_metas = data["img_metas"][0].data[0]
            imgs = tensor2imgs(img_tensor, **img_metas[0]["img_norm_cfg"])
            assert len(imgs) == len(img_metas)
            i = 0
            for img, img_meta in zip(imgs, img_metas):
                i += 1
                h, w, _ = img_meta["img_shape"]
                img_show = img[:h, :w, :]

                ori_h, ori_w = img_meta["ori_shape"][:-1]
                img_show = mmcv.imresize(img_show, (ori_w, ori_h))
                cv2.imwrite(depth_path + str(format(i, "04")) + ".png", img_show)

            prog_bar.update()
        return result_depths

    def infer_single_img(self, img):
        img = torch.tensor(img, device=self.device)
        prog_bar = mmcv.ProgressBar(1)
        with torch.no_grad():
            result_depth = self.model(return_loss=False, **img)
        prog_bar.update()
        return result_depth.cpu()
