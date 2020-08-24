import darknet

if __name__ == "__main__":
    #net = load_net("cfg/densenet201.cfg", "/home/pjreddie/trained/densenet201.weights", 0)
    #im = load_image("data/wolf.jpg", 0, 0)
    #meta = load_meta("cfg/imagenet1k.data")
    #r = classify(net, meta, im)
    #print r[:10]
    net = load_net("/home/nvidia/darknet/cfg/yolov3.cfg", "/home/nvidia/darknet/yolov3.weights", 0)
    meta = load_meta("/home/nvidia/darknet/cfg/coco.data")
    r = detect(net, meta, "/home/nvidia/darknet/data/dog.jpg")
    print(r)
