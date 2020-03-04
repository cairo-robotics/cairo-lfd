

class StaticRelativePositionGrounding(object):

    def __init__(self, environment, static_item_id):
        self.environment = environment
        self.item_id = static_item_id

    def transform(sample):
        static_object_pos = self.enviroment.get_item_state_by_id(self.item_id)
        x = static_object_pos[0] + sample[0]
        y = static_object_pos[1] + sample[1]
        z = static_object_pos[2] + sample[2]
        return [x, y, z]
