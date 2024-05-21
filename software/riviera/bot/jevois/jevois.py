import torch
import usb.core
import usb.util
import subprocess
import opencv


def find_id(auto=False) -> str:
    possible_ids = {"fron": 0, "left": 0.5, "back": 1, "righ": 1.5}
    try:
        with open("id.txt", "r") as f:
            line = f.readlines()[0].replace("\n", "")
    except Exception as e:
        print("problem:")
        print(e)
        line = ""
    if line in possible_ids:
        id_ = line
        print(f"ID-ed as : {id_}")
    else:
        print("ID NOT SET!!!")
        print(f"please specify one of {possible_ids}(orientation in pi)")
        if not auto:
            id_ = input("set id:")
            assert id_ in possible_ids, "please choose correct ID"
        else:
            id_ = "fron"
        try:
            with open("id.txt", mode="w") as f:
                f.write(id_)
        except Exception as e:
            print("could not write ID!", e)
    return id_


def find_nano():
    usbs = subprocess.check_output(["lsusb"]).decode("utf-8").split("\n")
    for usb_ in usbs:
        if "JETSON_NA=NO" in usb_:  # TODO CHANGE THIS
            id_pos = usb_.find("ID")
            id_ = usb_[id_pos + 3 : id_pos + 3 + 8 + 1]
    print("Jetson nano not detected!")
    id_ = "0000:0000"  # TODO change this
    print(f"using saved ID:{id_}")

    # find our device
    dev = usb.core.find(
        idVendor=int(id_[0:4], base=16), idProduct=int(id[5:9], base=16)
    )
    if dev is None:
        raise ValueError("Device not found")

    dev.set_configuration()

    # get an endpoint instance
    cfg = dev.get_active_configuration()
    intf = cfg[(0, 0)]

    ep = usb.util.find_descriptor(
        intf,
        # match the first OUT endpoint
        custom_match=lambda e: usb.util.endpoint_direction(e.bEndpointAddress)
        == usb.util.ENDPOINT_OUT,
    )

    assert ep is not None, "usb problem"
    return ep


if __name__=="__main__":
    ID = find_id(False)
    EP = find_nano()
    EP.write(f"{ID} here, ready to send data")
