import kg_robot as kgr
import teach_mode as tm

class specialised_kg_robot(kgr.kg_robot,tm.teach):
    # see kg_robot for connection initialisation params
    def __init__(self, port=False, ee_port=False, db_host=False, name='burt',test=False):
        kgr.kg_robot.__init__(self, port, ee_port, db_host, name)
        tm.teach.__init__(self)
        print(test)
    # setup up your specialised robot fns here

    def test(self):
        print('hi')
