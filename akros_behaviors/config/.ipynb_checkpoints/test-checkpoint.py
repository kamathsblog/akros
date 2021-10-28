import yaml

if __name__ == "__main__":
    new_dict = {
        'waypoint'+str(6) : {
            'x': 0,
            'y': 0,
            'theta': 0,
        }
    }

    with open('waypoints_test.yaml', 'r') as yamlfile:
        cur_yaml = yaml.safe_load(yamlfile)
        cur_yaml['waypoints'].update(new_dict)
        
    if cur_yaml:
        print(cur_yaml)
        with open('waypoints_test.yaml', 'w') as yamlfile:
            yaml.safe_dump(cur_yaml, yamlfile)
