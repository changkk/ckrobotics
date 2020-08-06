import darknet

if __name__ == "__main__":
    #net = load_net("cfg/densenet201.cfg", "/home/pjreddie/trained/densenet201.weights", 0)
    #im = load_image("data/wolf.jpg", 0, 0)
    #meta = load_meta("cfg/imagenet1k.data")
    #r = classify(net, meta, im)
    #print r[:10]
    net = load_net("/home/usl/darknet/cfg/yolov2.cfg", "/home/usl/darknet/yolov2.weights", 0)
    meta = load_meta("/home/usl/darknet/cfg/coco.data")
    r = detect(net, meta, "/home/usl/darknet/data/dog.jpg")
    print(r)
    