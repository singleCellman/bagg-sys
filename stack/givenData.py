import stack.configData as cfg

container_size = [cfg.spaceLength, cfg.spaceWidth, cfg.spaceHeight]

item_size_set = []
for i in range(cfg.baggageLength_min, cfg.baggageLength_max):
    for j in range(cfg.baggageWidth_min, cfg.baggageWidth_max):
        for k in range(cfg.baggageThick_min, cfg.baggageThick_max):
            item_size_set.append((i, j, k))
print(item_size_set)
