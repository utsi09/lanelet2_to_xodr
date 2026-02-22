import xml.etree.ElementTree as ET
import numpy as np
from math import sqrt, atan2
import sys

def parse_lanelet2_local(osm_file):
    tree = ET.parse(osm_file)
    root = tree.getroot()
    
    nodes = {}
    for node in root.findall('node'):
        node_id = node.get('id')
        local_x = None
        local_y = None
        for tag in node.findall('tag'):
            if tag.get('k') == 'local_x':
                local_x = float(tag.get('v'))
            elif tag.get('k') == 'local_y':
                local_y = float(tag.get('v'))
        
        if local_x is not None and local_y is not None:
            nodes[node_id] = {'x': local_x, 'y': local_y}
    
    ways = {}
    for way in root.findall('way'):
        way_id = way.get('id')
        node_refs = [nd.get('ref') for nd in way.findall('nd')]
        tags = {}
        for tag in way.findall('tag'):
            tags[tag.get('k')] = tag.get('v')
        
        ways[way_id] = {
            'nodes': node_refs,
            'tags': tags
        }
    
    relations = []
    for relation in root.findall('relation'):
        rel_type = None
        for tag in relation.findall('tag'):
            if tag.get('k') == 'type':
                rel_type = tag.get('v')
        
        if rel_type == 'lanelet':
            members = []
            for member in relation.findall('member'):
                members.append({
                    'type': member.get('type'),
                    'ref': member.get('ref'),
                    'role': member.get('role')
                })
            
            tags = {}
            for tag in relation.findall('tag'):
                tags[tag.get('k')] = tag.get('v')
            
            relations.append({
                'members': members,
                'tags': tags
            })
    
    return nodes, ways, relations

def calculate_geometry(points):
    geometries = []
    s_offset = 0.0
    
    for i in range(len(points) - 1):
        x1, y1 = points[i]['x'], points[i]['y']
        x2, y2 = points[i+1]['x'], points[i+1]['y']
        
        dx = x2 - x1
        dy = y2 - y1
        length = sqrt(dx*dx + dy*dy)
        
        if length > 0.01:
            heading = atan2(dy, dx)
            geometries.append({
                's': s_offset,
                'x': x1,
                'y': y1,
                'hdg': heading,
                'length': length
            })
            s_offset += length
    
    return geometries, s_offset

def create_opendrive_local(nodes, ways, relations, output_file):
    opendrive = ET.Element('OpenDRIVE')
    
    header = ET.SubElement(opendrive, 'header')
    header.set('revMajor', '1')
    header.set('revMinor', '4')
    header.set('name', 'Lanelet2 Converted Map')
    header.set('version', '1.0')
    header.set('date', '2024-01-01T00:00:00')
    header.set('north', '0')
    header.set('south', '0')
    header.set('east', '0')
    header.set('west', '0')
    
    road_id = 0
    processed_roads = 0
    
    for relation in relations:
        left_way = None
        right_way = None
        
        for member in relation['members']:
            if member['role'] == 'left' and member['type'] == 'way':
                left_way = member['ref']
            elif member['role'] == 'right' and member['type'] == 'way':
                right_way = member['ref']
        
        if left_way and right_way and left_way in ways and right_way in ways:
            left_nodes = ways[left_way]['nodes']
            right_nodes = ways[right_way]['nodes']
            
            left_points = [nodes[nid] for nid in left_nodes if nid in nodes]
            right_points = [nodes[nid] for nid in right_nodes if nid in nodes]
            
            if len(left_points) > 1 and len(right_points) > 1:
                centerline_points = []
                min_len = min(len(left_points), len(right_points))
                
                for i in range(min_len):
                    center_x = (left_points[i]['x'] + right_points[i]['x']) / 2
                    center_y = (left_points[i]['y'] + right_points[i]['y']) / 2
                    centerline_points.append({'x': center_x, 'y': center_y})
                
                if len(centerline_points) > 1:
                    road = ET.SubElement(opendrive, 'road')
                    road.set('name', f'Road_{road_id}')
                    road.set('id', str(road_id))
                    road.set('junction', '-1')
                    
                    geometries, total_length = calculate_geometry(centerline_points)
                    road.set('length', str(total_length))
                    
                    plan_view = ET.SubElement(road, 'planView')
                    for geom in geometries:
                        geometry = ET.SubElement(plan_view, 'geometry')
                        geometry.set('s', str(geom['s']))
                        geometry.set('x', str(geom['x']))
                        geometry.set('y', str(geom['y']))
                        geometry.set('hdg', str(geom['hdg']))
                        geometry.set('length', str(geom['length']))
                        ET.SubElement(geometry, 'line')
                    
                    elevation_profile = ET.SubElement(road, 'elevationProfile')
                    elevation = ET.SubElement(elevation_profile, 'elevation')
                    for attr in ['s', 'a', 'b', 'c', 'd']:
                        elevation.set(attr, '0')
                    
                    lanes = ET.SubElement(road, 'lanes')
                    lane_section = ET.SubElement(lanes, 'laneSection')
                    lane_section.set('s', '0')
                    
                    center = ET.SubElement(lane_section, 'center')
                    lane0 = ET.SubElement(center, 'lane')
                    lane0.set('id', '0')
                    lane0.set('type', 'none')
                    
                    lane_widths = []
                    for i in range(min_len):
                        dx = left_points[i]['x'] - right_points[i]['x']
                        dy = left_points[i]['y'] - right_points[i]['y']
                        lane_widths.append(sqrt(dx*dx + dy*dy))
                    avg_width = np.mean(lane_widths) if lane_widths else 3.5
                    
                    right = ET.SubElement(lane_section, 'right')
                    lane = ET.SubElement(right, 'lane')
                    lane.set('id', '-1')
                    lane.set('type', 'driving')
                    lane.set('level', 'false')
                    
                    width = ET.SubElement(lane, 'width')
                    width.set('sOffset', '0')
                    width.set('a', str(avg_width))
                    width.set('b', '0')
                    width.set('c', '0')
                    width.set('d', '0')
                    
                    road_mark = ET.SubElement(lane, 'roadMark')
                    road_mark.set('sOffset', '0')
                    road_mark.set('type', 'solid')
                    road_mark.set('weight', 'standard')
                    road_mark.set('color', 'white')
                    road_mark.set('width', '0.12')
                    
                    road_id += 1
                    processed_roads += 1
    
    tree = ET.ElementTree(opendrive)
    ET.indent(tree, space='  ')
    tree.write(output_file, encoding='utf-8', xml_declaration=True)
    return processed_roads

def main():
    input_file = "/home/taewook/Documents/Busan_Test/Assets/lanelet20902.osm"
    output_file = "/home/taewook/Documents/Busan_Test/Assets/lanelet20902.xodr"
    
    nodes, ways, relations = parse_lanelet2_local(input_file)
    processed_roads = create_opendrive_local(nodes, ways, relations, output_file)

if __name__ == "__main__":
    main()
