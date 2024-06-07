# This software is provided 'as-is', without any express or implied warranty. 
# In no event will the author be held liable for any damages arising from the 
# use of this software.
#
# Permission is granted to anyone to use this software for any purpose, 
# including commercial applications, and to alter it and redistribute it
# freely, subject to the following restrictions:
from spinapi import *
import copy
import matplotlib.pyplot as plt

class PulseBlasterUSB():
    def __init__(self, board_number=0, clock = 500, **kwargs):
        self.Initialize(board_number, clock)
        #This is useful for experiments where the pulse sequence needs to be changed mid experiment. You can now
        #initialize/optimize all experiments beforehand and then index the experiment quickly mid-experiment.
        #Also possible to combine buffer sequences.
        self.sequence_set = []
        #This one is just used to hold temporary sequences to pull from for organizing by the user when designing larger structures.
        self.temp_sequence_set = []
        self.sequence_start_time = 0
        self.sequence_time_length = 0
    
    #Simply accesses the spinapi python wrapper to communicate with the file "spinapi.h" which tells how to navigtate the
    #ctype control protocals in spincore's usual API for their controllers. Usually board 0 is sufficient.
    def Initialize(self, board_num, clock):
        pb_set_debug(board_num)
        pb_select_board(board_num)
        #Note for future reader: if the spinapi.py works but the library fails to initialize, make sure that the spinapi.h file
        #is in the correct directory. (For ubuntu(probably general linux): //usr/include). If it is there and still doesn't work,
        #go into the spinapi.h file and switch the line of ctypes.CDLL("spinapi") to:
        #ctypes.CDLL("path_to_spinapi_driver/build/src/libspinapi.so")
        if pb_init() != board_num:
            print("Error initializing board: %s" % pb_get_error())
            input("Please press a key to continue.")
            exit(-1)
        pb_core_clock(clock)
        
    #Adds a pulse sequence to the end of the buffer.
    def Add_Buffer_Sequence(self, inst_list):
        self.sequence_set += [inst_list]
        return len(self.sequence_set)-1
    
    #Used to clear the buffer entirely.        
    def Clear_Buffer(self):
        self.sequence_set = []
    
    #Note that the index of any sequence previously noted will be shifted down one if it is higher on the list.
    def Buffer_Delete(self, index):
        del self.sequence_set[index]
    
    #Stops the current pulse program (all channels level 0).
    def Stop(self):
        pb_stop()
    
    #Same as Stop(), but also disrups the connection with the device. Should be used when leaving a session to avoid locking, though
    #not using it is fixable by simply power-cycling the device if it locks.
    def Close(self):
        pb_stop()
        pb_close()
    
    #This command organizes the pulse sequence and optimizes the set of instructions. Structure example:
    #chan_list should have integer channel numbers in use corresponding to the list in d_list and p_list. E.g:
    #chan_list = [0, 1], d_list = [[20, 10, 500, 5000], [5500, 30]], p_list = [[1, 0, 0, 1], [1, 0]]
    #This gives ch0: on for 20ns, off for 10ns, off for 500ns, on for 5us.
    #ch1: on for 5.5us, off for 30ns
    #Note that by the end of the optimization, this pulse sequence will be 5 instructions long.
    #Experiment index is used to replace a pulse sequence in the buffer sequence set. Leave as a half to add it to the end.
    def get_Instructions(self, c_list, dur_list, pow_list, temp_buffer = ""):
        #These avoid issues of effecting the original lists supplied.
        chan_list = copy.deepcopy(c_list)
        d_list = copy.deepcopy(dur_list)
        p_list = copy.deepcopy(pow_list)
        #Instructions is [[channel bits],[instruction],[inst args (just 0)],[delay(ns)]]
        inst_list = [[],[],[],[]]
        #Checks if we are only running one channel, as if we are, then we can segment differently (faster)
        if len(chan_list) == 1:
            need_segmenting = False
            #This code doesn't both removing repetition of 0's and 1's as it will have little to no timing effect in just 1 channel.
            for i in list(range(len(p_list))):
                inst_list[0] += [p_list[i]*(2**chan_list[0])]
            #Always set args to 0 for our purposes
            inst_list[2] = [0]*len(p_list)
            #0 = continue in spincore.py
            inst_list[1] = [0]*len(p_list)
            #Sets last instruction to branching
            inst_list[1][-1] = 6
            inst_list[3] = d_list
        else:
            need_segmenting = True
        #Checks to make sure all channels list the same pulse run time to aid in avoiding mistakes.
        #Also combines delay times of instructions where the power is the same on 2 channels. This avoids unnessisary commands.
        #eg d_list = [[10, 10], [2, 4, 6, 8]] and p_list = [[0, 0], [0, 1, 1, 0]]
        #becomes d_list = [[20], [2, 10, 8]] and p_list = [[0],[0, 1, 0]]
        if need_segmenting:
            sum_delays = round(sum(d_list[0]), 12)
            for i in list(range(len(d_list))):
                if sum_delays != round(sum(d_list[i]), 12):
                    print(f"Mismatch of pulse timing found at channel {chan_list[i]}. Make sure all channels have the same length of pulse sequence.")
                    raise ErrorValue()
                #Gets last index for easier parsing
                chan_len = len(d_list[i]) - 1
                #q keeps track of number of deleted indecies for properly managing the list.
                q = 0
                #Removes repeated power levels for command density
                for j in list(range(len(d_list[i]))):
                    if j < chan_len:
                        if p_list[i][j-q] == p_list[i][j+1-q]:
                            d_list[i][j-q] += d_list[i][j+1-q]
                            del p_list[i][j-q+1]
                            del d_list[i][j-q+1]
                            q += 1
        #Segments the channels into format needed for processing quickly.
        while need_segmenting == True:
            next_end = d_list[0][0]
            next_end_index = 0
            for i in list(range(1, len(d_list))):
                #Makes sure that if 2 channels ended at the same time, we aren't adding a 0-length pulse
                if round(d_list[i][0], 9) == 0: #Round to avoid floating point issues
                    del d_list[i][0]
                    del p_list[i][0]
                #Gets the next instruction's delay time as next lowest delay. Saves index of lowest for deletion.
                if round(d_list[i][0], 9) <= round(next_end, 9):
                    next_end = d_list[i][0]
                    next_end_index = i
            #Below organizes the next instruction segmenting.
            #Organizes the channels, noting that 2^(ch#) in instruction activates that channel for the given delay. Power says if on or off.
            inst_list[0] += [0]
            for i in list(range(len(chan_list))):
                inst_list[0][-1] += p_list[i][0]*(2**chan_list[i])
            #organizes the delay and sets up next iteration.
            for i in list(range(len(d_list))):
                if i == next_end_index or round(d_list[i][0] - next_end, 9) == 0: #Deletes known channel's index
                    del d_list[i][0]
                    del p_list[i][0]
                else:
                    d_list[i][0] = round(d_list[i][0] - next_end, 9) #Removes floating point issues
            if round(next_end, 9) < 1e-8:
                print("error: pulse too short.")
                raise ErrorValue()
            else:
                inst_list[3] += [round((next_end), 12)] #avoids floating point errors
            #Adds the instructions and arguments. Arguments always 0.
            inst_list[2] += [0]
            #Note this structure gives branching(6) for the last case always and continue(1) for the rest.
            if len(d_list[0]) == 0 or (len(d_list[0]) == 1 and d_list[0][0] == 0):
                inst_list[1] += [6] #Branch
                need_segmenting = False
            else:
                inst_list[1] += [0] #Continue
        if temp_buffer == "B" or temp_buffer == "b":
            return self.Add_Buffer_Sequence(inst_list)
        elif temp_buffer == "T" or temp_buffer == "t":
            return self.Temp_Add(inst_list)
        else:
            return inst_list 
    
    #Used to loop over an instruction set and combine them. (ADD A WAY TO REMOVE DUPLICATE PULSES WHEN START AND END ARE THE SAME)
    def Loop(self, inst_list, loops):
        new_inst_list = [[], [], [], []]
        if inst_list[0][0] == inst_list[0][-1]:
            #Removes last entry, which is combined with the first term of middle.
            new_inst_list = [inst_list[0][:-1],inst_list[1][:-1],inst_list[2][:-1],inst_list[3][:-1]]
            middle = [inst_list[0][:-1],inst_list[1][:-1],inst_list[2][:-1],inst_list[3][:-1]]
            middle[3][0] += inst_list[3][-1]
            for i in list(range(loops - 2)):
                new_inst_list[0] += middle[0]
                new_inst_list[1] += middle[1]
                new_inst_list[2] += middle[2]
                new_inst_list[3] += middle[3]
            end = [inst_list[0],inst_list[1],inst_list[2],inst_list[3]]
            end[3][0] += inst_list[3][-1]
            #This lets us add the last block with the first instruction absorbed in the previous and the last instruction added to the end.
            #Note this structure also automatically handles the last argument being branching rather then continuous.
            new_inst_list[0] += end[0]
            new_inst_list[1] += end[1]
            new_inst_list[2] += end[2]
            new_inst_list[3] += end[3]
        else:
            for i in list(range(loops)):
                new_inst_list[0] += inst_list[0]
                new_inst_list[1] += [0]*len(inst_list[1]) #All except the last will be continuous (branching changed at the end.
                new_inst_list[2] += inst_list[2]
                new_inst_list[3] += inst_list[3]
            new_inst_list[1][-1] = 6 #Sets last to branching
        return new_inst_list
    
    #Used to combine a set of pulse sequence send to end from the sequence buffer.
    #to_buffer tells whether to tack the sequence onto the end of the sequence_set (default) or if it simply returns the combined sequence.
    def New_Sequence_From_Buffer(self, index_list, temp_buffer = ""):
        new_inst_list = [[],[],[],[]]
        for index in index_list:
            #used to check for looping. Should be the form: [index, N] where N = number of loops.
            #Can also be of the form [[index_1, index_2, ..., index_n], N] for looping over a combination of sequences in the buffer.
            #loops the sequence at the first index of sequence_set (the buffer) 5 times.
            if type(index) == list:
                if type(index[0]) == list:
                    #Loops over the same instruction and adds it collectively. Note loop optimizes matching start and end conditions.
                    inst_list = self.Loop(self.New_Sequence_From_Buffer(index[0], False), index[1])
                else:
                    inst_list = self.Loop(self.sequence_set[index[0]], index[1])
            #If not looping, just use the sequence index. If an error here is found, you're using bad indexing in your input.
            else:
                inst_list = self.sequence_set[index]
            #Simply combines the list.
            new_inst_list[0] += inst_list[0]
            new_inst_list[1] += [0]*len(inst_list[2]) #All except the last will be continuous (branching changed at the end.
            new_inst_list[2] += inst_list[2]
            new_inst_list[3] += inst_list[3]
        new_inst_list[1][-1] = 6 #Sets last to branching
        if temp_buffer == "B" or temp_buffer == "b":
            return self.Add_Buffer_Sequence(new_inst_list)
        elif temp_buffer == "T" or temp_buffer == "t":
            return self.Temp_Add(new_inst_list)
        else:
            return new_inst_list
    
    #This is similar to New_Sequence_From_Buffer, but for sequence handeling without the buffer. Note that by default, it returns the
    #sequence of interest as a new sequence. To save to buffer or to temp, these need to be turned to True.
    #Actions should be a list refering to the indicies in instruction list for pulling instructons and using loop structures.
    #e.g. for a 3 different pulse sequence instructions long list, an acceptible actions would be:
    #actions = [0, [1, 2], 2], which would be instruction 0 followed by instruction 1 twice, then instruction 2.
    def New_Sequence(self, instructions_list, actions, temp_buffer = ""):
        new_inst_list = [[], [], [], []]
        for action in actions:
            if type(action) == int:
                inst_list = instructions_list[action]
            elif type(action) == list:
                if len(action) != 1:
                    inst_list = self.Loop(instructions_list[action[0]], action[1])
                #This avoids if someone accidently does [0, [1], 2] when meaning [0, 1, 2]
                else:
                    inst_list = instructions_list[action[0]]
            #Simply combines the list.
            new_inst_list[0] += inst_list[0]
            new_inst_list[1] += [0]*len(inst_list[2]) #All except the last will be continuous (branching changed at the end.
            new_inst_list[2] += inst_list[2]
            new_inst_list[3] += inst_list[3]
        new_inst_list[1][-1] = 6 #Sets last to branching
        if temp_buffer == "B" or temp_buffer == "b":
            return self.Add_Buffer_Sequence(new_inst_list)
        elif temp_buffer == "T" or temp_buffer == "t":
            return self.Temp_Add(new_inst_list)
        else:
            return new_inst_list      
                
    #New_Sequence_From_Buffer, but for sequences from the temp list.
    def New_Sequence_From_Temp(self, index_list, temp_buffer = ""):
        new_inst_list = [[],[],[],[]]
        for index in index_list:
            #used to check for looping. Should be the form: [index, N] where N = number of loops.
            #Can also be of the form [[index_1, index_2, ..., index_n], N] for looping over a combination of sequences in the buffer.
            #loops the sequence at the first index of sequence_set (the buffer) 5 times.
            if type(index) == list:
                if type(index[0]) == list:
                    #Loops over the same instruction and adds it collectively. Note loop optimizes matching start and end conditions.
                    inst_list = self.Loop(self.New_Sequence_From_Temp(index[0], False), index[1])
                else:
                    inst_list = self.Loop(self.temp_sequence_set[index[0]], index[1])
            #If not looping, just use the sequence index. If an error here is found, you're using bad indexing in your input.
            else:
                inst_list = self.temp_sequence_set[index]
            #Simply combines the list.
            new_inst_list[0] += inst_list[0]
            new_inst_list[1] += [0]*len(inst_list[2]) #All except the last will be continuous (branching changed at the end.
            new_inst_list[2] += inst_list[2]
            new_inst_list[3] += inst_list[3]
        new_inst_list[1][-1] = 6 #Sets last to branching
        if temp_buffer == "B" or temp_buffer == "b":
            return self.Add_Buffer_Sequence(new_inst_list)
        elif temp_buffer == "T" or temp_buffer == "t":
            return self.Temp_Add(new_inst_list)
        else:
            return new_inst_list
    
    #Uses the instructions structure made by the get_Instructions() function to rapidly load the desired sequence into the pulseblaster.
    #Designed to be as efficient as is possible when using python through pulse optimization prior to flushing.
    def Send_Instructions(self, instruction_list):
        start = pb_inst_pbonly(instruction_list[0][0], instruction_list[1][0], instruction_list[2][0], instruction_list[3][0]*1e9)
        for i in list(range(1, len(instruction_list[0]) - 1)):
            pb_inst_pbonly(instruction_list[0][i], instruction_list[1][i], instruction_list[2][i], instruction_list[3][i]*1e9)
        pb_inst_pbonly(instruction_list[0][-1], instruction_list[1][-1], start, instruction_list[3][-1]*1e9)
    
    #Runs the sequence at a specific index of the buffer. If no index is specified, then it just runs the last one in the list.
    #By default, this will wait for the last pulse sequence to finish it's itteration before starting the new instructions.
    #To force it to start the next instructions immediately, set reset to True.
    def Run_Sequence(self, sequence, reset = False):
        pb_start_programming(PULSE_PROGRAM)
        self.Send_Instructions(sequence)
        pb_stop_programming()
        if reset:
            pb_reset()
        pb_start()
    
    #Runs the sequence at a specific index of the buffer. If no index is specified, then it just runs the last one in the list.
    #By default, this will wait for the last pulse sequence to finish it's itteration before starting the new instructions.
    #To force it to start the next instructions immediately, set reset to True.
    def Run_Buffer_Sequence(self, index = -1, reset = False):
        pb_start_programming(PULSE_PROGRAM)
        self.Send_Instructions(self.sequence_set[index])
        pb_stop_programming()
        if reset:
            pb_reset()
        pb_start()
    
    #Adds a sequence to the temp_sequence_set variable and returns the index that it is stored in.
    #Note: if an index is provided, it will insert the sequence at that index. If it isn't, then it will be tacked to the end.
    def Temp_Add(self, sequence, index = 0.5):
        #Default adds the sequence to the end if not specified.
        if index == 0.5:
            self.temp_sequence_set.append(sequence)
        else:
            self.temp_sequence_set.insert(index, sequence)
        if index == 0.5:
            return len(self.temp_sequence_set) - 1
        elif index >= 0:
            return index
        else:
            return len(self.temp_sequence_set) + index
    
     #Adds a sequence to the temp_sequence_set variable and returns the index that it is stored in.
    #Note: if an index is provided, it will insert the sequence at that index. If it isn't, then it will be tacked to the end.
    def Buffer_Add(self, sequence, index = 0.5):
        #Default adds the sequence to the end if not specified.
        if index == 0.5:
            self.sequence_set.append(sequence)
        else:
            self.sequence_set.insert(index, sequence)
        if index == 0.5:
            return len(self.sequence_set) - 1
        elif index >= 0:
            return index
        else:
            return len(self.sequence_set) + index
    
    #Removes the sequence at the provided index from temp_sequence_set
    def Temp_Delete(self, index):
        del self.temp_sequence_set[index]
    
    #Replaces the sequence at a specific index of the temperary buffer with a different one.
    def Temp_Replace(self, sequence, index):
        self.temp_sequence_set[index] = sequence
        
    #Replaces the sequence at a specific index of the temperary buffer with a different one.
    def Buffer_Replace(self, sequence, index):
        self.temp_sequence_set[index] = sequence
    
    #Sets temp_sequence_set to an empty list.
    def Clear_Temp(self):
        self.temp_sequence_set = []
    
    #gets a specific index from temp_sequence_set
    def get_Temp_Sequence(self, index = -1):
        return self.temp_sequence_set[index]
    
    #Returns the whole list of temp_sequence_set.
    def get_Temp(self):
        return self.temp_sequence_set
    
    #Returns the whole list of sequence_set.
    def get_Buffer(self):
        return self.sequence_set
    
    #Determines the channels used in a given pulse sequence
    def List_Channels_Used(self, sequence):
        #Gets largest combined channel number from the sequence, then converts to bionary, where the largest channel is nessisarily
        #included in the largest number. length of the bionary number minus 1 gives the largest channel number.
        max_chan_num = len(f"{max(sequence[0]):b}")-1
        used_channels = [max_chan_num]
        unused_channels = list(range(max_chan_num))
        i = 0
        #Gets a list of the channels that are used in the sequence.
        while len(unused_channels) != 0:
            cur_channels = f"{sequence[0][i]:b}"
            #Need this since the largest bionary diget of cur_channels is indexed at the 0th index, so we need to work backwards.
            delete_val = []
            length = len(f"{sequence[0][i]:b}") - 1
            for j in list(range(len(unused_channels))):
                #Doesn't bother checking if all channels are off in this step.
                if sequence[0][i] == 0:
                    break
                if cur_channels[length - unused_channels[j]] == "1" and length-unused_channels[j] >= 0:
                    used_channels += [int(unused_channels[j])]
                    delete_val += [unused_channels[j]]
            for val in delete_val:
                unused_channels.remove(val)
            i += 1
            if i == len(sequence[0]):
                break
        used_channels.sort()
        return used_channels
    
    #Plots the sequence in all channels used within the sequence.
    def Plot_Sequence(self, sequence):
        used_channels = self.List_Channels_Used(sequence)
        for chan in used_channels:
            if chan == used_channels[-1]:
                is_Last = True
            else:
                is_Last = False
            cur_time = 0
            time_list = [0] #Time in seconds
            level_list = [0] #1 = on, 0 = off
            for i in list(range(len(sequence[0]))):
                cur_chan_in_bi = f"{sequence[0][i]:b}"
                #Checks if it's 0, as we can't look through all the values. The is_Last and associated check is for the case of the last bit,
                #as the lowest channel will not always have an index.
                if sequence[0][i] == 0 or len(cur_chan_in_bi) - chan - 1 < 0 or cur_chan_in_bi[len(cur_chan_in_bi) - chan - 1] == "0":
                    level_list += [0, 0]
                    time_list += [time_list[-1] + 2e-9] #Graphs pulse with rise time of 2ns (based on 500MHz clock frequency)
                    time_list += [time_list[-2] + sequence[3][i]] #Adds the full duration of off to the sequence.
                else:
                    level_list += [1, 1]
                    time_list += [time_list[-1] + 2e-9] #Graphs pulse with rise time of 2ns (based on 500MHz clock frequency)
                    time_list += [time_list[-2] + sequence[3][i]] #Adds the full duration of on to the sequence.
            plt.plot(time_list, level_list)
            plt.xlabel('Time(s)')
            plt.ylabel('Level')
            plt.title(f'Ch{chan}')
            plt.show()
    #Gets the buffer sequence from a specific index. Default is the last sequence in sequence_set.
    def get_Buffer_Sequence(self, index = -1):
        return self.sequence_set[index]
