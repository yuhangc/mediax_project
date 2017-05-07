#!/usr/bin/env python

import numpy as np
import json


def generate_actions(total_num, action_list):
    num_actions = len(action_list)
    if total_num % num_actions != 0:
        raise Exception("Number of training/trials should be multiple of actions!")

    num_rep = total_num / num_actions
    actions = []

    for act in range(num_actions):
        actions += [action_list[act]] * num_rep

    return list(np.random.permutation(actions))


def generate_protocol(num_cond, num_trainings, num_trials, action_list):
    """
    Generate actions for each trial based on provided information
    :param num_cond: number of conditions
    :param num_trainings: number of trainings in each condition
    :param num_trials: number of trials in each condition
    :param action_list: list of valid actions
    :return: list of actions for each condition
    """
    action_protocol = []

    for cond in range(num_cond):
        # generate training actions
        action_trainings = generate_actions(num_trainings[cond], action_list)

        # generate trial actions
        action_trials = generate_actions(num_trials[cond], action_list)

        action_protocol.append(action_trainings + action_trials)

    return action_protocol


def save_protocol(file_name, num_cond, num_trainings, num_trials, action_protocol):
    # create an object that stores the protocols
    protocol_obj = {}
    protocol_obj["num_conditions"] = num_cond
    protocol_obj["num_trainings"] = num_trainings
    protocol_obj["num_trials"] = num_trials

    protocol_obj["robot_actions"] = {}
    for i in range(num_cond):
        cond_name = "condition_" + str(i)
        protocol_obj["robot_actions"][cond_name] = action_protocol[i]

    with open(file_name, 'w') as out_file:
        json.dump(protocol_obj, out_file)


if __name__ == "__main__":
    num_cond = int(input("Please enter the total number of conditions: "))

    separator = " "
    num_trainings = map(int, raw_input("Enter the number of trainings for each condition: ").split(separator))
    num_trials = map(int, raw_input("Enter the number of trials for each condition: ").split(separator))
    action_list = map(int, raw_input("Enter valid action ids: ").split(separator))

    print "Generating protocol..."
    protocol = generate_protocol(num_cond, num_trainings, num_trials, action_list)
    print protocol

    file_name = raw_input("Enter protocol file name: ")
    print "Saving protocol..."
    save_protocol(file_name, num_cond, num_trainings, num_trials, protocol)
