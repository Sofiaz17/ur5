import os
import rospkg
import random
import xml.etree.ElementTree as et
import lxml.etree as etree

# TODO bisogna scrivere dentro il file xml world.world (in qualche modo)
# + manca la selezione casuale della classe dei blocchetti LEGO

# ----------------------- CONSTANTS -----------------------
#WORLD_PATH = rospkg.RosPack().get_path('ros_impedance_controller')+'/worlds/'
WORLD_FILE = '../robotics_project_ur5/world.world'

MIN_NUM_BLOCKS = 3

LEGO_LABELS = [ 'X1-Y1-Z2',
                'X1-Y2-Z1',
                'X1-Y2-Z2',
                'X1-Y2-Z2-CHAMFER',
                'X1-Y2-Z2-TWINFILLET',
                'X1-Y3-Z2',
                'X1-Y3-Z2-FILLET',
                'X1-Y4-Z1',
                'X1-Y4-Z2',
                'X2-Y2-Z2',
                'X2-Y2-Z2-FILLET']

# ----------------------- GENERATION PARAMS -----------------------
MIN_X = 0
MAX_X = 0.5

MIN_Y = 0.2
MAX_Y = 0.8


# ----------------------- FUNCTIONS -----------------------


def generate_position():
    # generates a pair of coordinates on the table
    x = round(random.uniform(MIN_X, MAX_X), 2)
    y = round(random.uniform(MIN_Y, MAX_Y), 2)

    new_pos = str(x) + ' ' + str(y) + ' 0.9 0 0 0'

    return new_pos


def is_occupied_position(new_pos, blocks_on_table):

    for current_pos in blocks_on_table:
        if current_pos == new_pos:
            return True

    return False


def generate_blocks(root):
    # Function that generates block disposition and type on the table

    #xmlFile = etree.tostring(root, pretty_print=True, xml_declaration=True, encoding='UTF-8')

    blocks_positions=[]

    # TODO need to synchronize?
    i = 0 # iteration
    for pos in root.iter('pose'):
        if i < 2: # skipping table position
            i += 1
            continue

        new_pos = generate_position()

        # checking whether there is another block in that position
        while is_occupied_position(new_pos, blocks_positions):
            new_pos = generate_position()
        # storing new block position
        pos.text = new_pos
        blocks_positions.append(pos.text)

        # choose random type of the blocks
        block_type = random.choice(LEGO_LABELS)



        i+=1

        #print("Block: " + )
# ----------------------- MAIN -----------------------

if __name__ == '__main__':

    # Generate the number of blocks in the world
    num_of_objects = random.randint(MIN_NUM_BLOCKS, len(LEGO_LABELS))

    print("Number of objects: ", num_of_objects)

    #path_tree = ET.parse(WORLD_PATH+'lego.world')   #TODO CHECK THIS FILE

    parser = et.parse('empty.world')
    parser.write('world.world')

    world_file = et.parse('../world.world')
    world = world_file.find(".//*[name='default']")

    new_block = et.Element("include")

    name = et.SubElement(new_block, "name")
    uri = et.SubElement(new_block, "uri")
    pose = et.SubElement(new_block, "pose")

    tree = etree.parse("world.world")
    root = tree.getroot()
    world = root.getchildren()
    world.append(new_block)
    print(etree.tostring(new_block, pretty_print=True).decode("utf-8"))



    #generate_blocks(root)
    #path_tree.write(WORLD_PATH+'lego.world')