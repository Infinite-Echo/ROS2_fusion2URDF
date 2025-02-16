from xml.etree.ElementTree import Element

class Property(Element):
    def __init__(self, name: str, value: str):
        super().__init__('xacro:property', attrib={'name':name, 'value':value})
    
class PropertyBlock(Element):
    def __init__(self, name: str):
        super().__init__('xacro:property', attrib={'name':name})

class InsertBlock(Element):
    def __init__(self, name: str):
        super().__init__('xacro:insert_block', attrib={'name':name})

class If(Element):
    def __init__(self, value: str):
        super().__init__('xacro:if', attrib={'value':value})

class Include(Element):
    def __init__(self, filename: str):
        super().__init__('xacro:include', attrib={'filename':filename})
