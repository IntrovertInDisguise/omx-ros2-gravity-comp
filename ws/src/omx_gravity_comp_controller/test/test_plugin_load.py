import os
import xml.etree.ElementTree as ET


def test_plugin_xml_contents():
    # plugin.xml is installed in share/omx_gravity_comp_controller but we can read the source copy
    pkg_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
    xml_path = os.path.join(pkg_dir, 'plugin.xml')
    assert os.path.exists(xml_path), f"plugin.xml not found at {xml_path}"

    tree = ET.parse(xml_path)
    root = tree.getroot()
    classes = root.findall('.//class')
    assert classes, "No <class> elements found in plugin.xml"

    # check for our known controller entry
    found = False
    for c in classes:
        name = c.get('name')
        typ = c.get('type')
        if name == 'omx_gravity_comp_controller/OmxGravityCompController':
            found = True
            assert typ.startswith('omx_gravity_comp_controller::'), "type attribute has unexpected prefix"
            break
    assert found, "Expected gravity compensation controller class not listed in plugin.xml"
