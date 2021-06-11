class REFEREE():
    def __init__(self,player_list):
        self.player_list = player_list           # 元素为[(left, top, right, bottom), pred_index, confidence]
        self.position_threshold   = 250         # 上下层货架的分界线.小于，则是上层; 大于，则是下层
        # self.confidence_threshold = 0.75         # 置信度的分界线
        self.confidence_threshold = 0.8         # 置信度的分界线


    # 同一层货架里只要出现一个置信度不满足，则认为是干扰物
    def judge(self):
        self.tmp_result = [[0,0],[0,0]]    # [ [上层货架目标货物数, 有无可疑物], [下层货架目标货物数, 有无可疑物] ]
        
        for i,v in enumerate(self.player_list):

            vertical_center = ( v[0][1] + v[0][3] ) / 2       # 使用top and bottom 判断
            # vertical_center = ( v[0][0] + v[0][2] ) / 2         # 使用left and right 判断


            if vertical_center < self.position_threshold:
                # 是上层货物
                if v[2] >= self.confidence_threshold:
                    self.tmp_result[0][0] += 1
                else:
                    # 出现可疑物
                    self.tmp_result[0][1] = 1

            else:
                # 是下层货物
                if v[2] >= self.confidence_threshold:
                    self.tmp_result[1][0] += 1
                else:
                    # 出现可疑物
                    self.tmp_result[1][1] = 1
            
        self.result = [0,0] # [上层货架目标货物数, 下层货架目标货物数]
        
        if self.tmp_result[0][1] == 0:
            self.result[0] = self.tmp_result[0][0]
        
        if self.tmp_result[1][1] == 0:
            self.result[1] = self.tmp_result[1][0]
        
        return self.result


if __name__ == "__main__":
    player_list = [[(0, 50, 0, 100), 0, 0.995],[(0, 50, 0, 100), 0, 0.8],[(0, 150, 0, 150), 0, 0.999]]
    refree = REFEREE(player_list)
    print(refree.judge())