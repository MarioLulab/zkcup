# package for ssd and cnn
from PIL import Image
from ssd import SSD
from cnn import CNN
from referee import REFEREE


class MODEL(object):
    def __init__(self):
        self.object_crop_list = []

    def generate(self):
        self.cnn = CNN()
        self.ssd = SSD()

    def predict(self, image):
        r_image,self.object_crop_list = self.ssd.detect_image(image)
        player_list = []

        for i,v in enumerate(self.object_crop_list):

            region = image.crop(v)

            preds_result, confidence = self.cnn.detect_image(region)

            # if preds_result == 0 or preds_result == 3:
            #     # 不抓取红牛和啤酒,置信度置为0.067
            #     player_list.append([v,preds_result,0.067])
            # else:
            #     player_list.append([v,preds_result,confidence])

            player_list.append([v,preds_result,confidence])


            print(self.cnn.class_names[preds_result])
            print(confidence)
            print("***************")
        
        # print(player_list)

        refree = REFEREE(player_list)   # 创建判断器. 注意修改其中的position_threshold和confidence_threshold
        result = refree.judge()
        print(result)

        return result   # [上层货架目标货物数，下层货架目标货物数]

    

