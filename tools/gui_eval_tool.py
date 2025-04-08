import os
import tkinter as tk
from tkinter import ttk

from pypibt import (
    PIBT,
    get_grid,
    get_scenario,
    is_valid_mapf_solution,
    save_configs_for_visualizer,
    cost_of_solution
)

class FileFilterTool:
    def __init__(self, root):
        self.root = root
        self.root.title("File Filter Tool")
        
        # Create frames for the list views
        self.map_frame = ttk.LabelFrame(root, text="*.map Files")
        self.map_frame.grid(row=0, column=0, padx=10, pady=10, sticky="nsew")
        
        self.scen_frame = ttk.LabelFrame(root, text="*.scen Files")
        self.scen_frame.grid(row=0, column=1, padx=10, pady=10, sticky="nsew")
        
        # Create listboxes
        self.map_listbox = tk.Listbox(self.map_frame, width=40, height=20)
        self.map_listbox.pack(fill="both", expand=True, padx=5, pady=5)
        
        self.scen_listbox = tk.Listbox(self.scen_frame, width=40, height=20)
        self.scen_listbox.pack(fill="both", expand=True, padx=5, pady=5)
        
        # Populate the listboxes
        self.populate_listboxes()
        # Bind selection event to map_listbox
        self.map_listbox.bind("<<ListboxSelect>>", self.update_scen_listbox)
        self.scen_listbox.bind("<<ListboxSelect>>", self.update_scen_selection)

        # Add a number input for the number of agents
        self.agent_frame = ttk.LabelFrame(root, text="Number of Agents")
        self.agent_frame.grid(row=2, column=0, columnspan=2, padx=10, pady=10, sticky="nsew")

        self.agent_spinbox = ttk.Spinbox(self.agent_frame, from_=1, to=100, width=10)
        self.agent_spinbox.pack(padx=5, pady=5)
        self.agent_spinbox.set(1)  # Set default value to 1
        # Add a button to print selections in both listboxes
        self.run_button = ttk.Button(root, text="Run", command=self.print_selections)
        self.run_button.grid(row=1, column=0, columnspan=1, pady=10)
      
        self.refresh_button = ttk.Button(root, text="Refresh", command=self.refresh_listboxes)
        self.refresh_button.grid(row=1, column=1, columnspan=1, pady=10)

        self.selected_map = None
        self.selected_scen = None

      
    def print_selections(self):
        # Get selected items from map_listbox
        selected_maps = self.map_listbox.get(self.selected_map[0])
        # Get selected items from scen_listbox
        selected_scens = self.selected_scen
        
        # Print the selections
        print("Selected .map files:", selected_maps)
        print("Selected .scen files:", selected_scens)

        grid = get_grid(os.path.join("assets", selected_maps))
        starts, goals = get_scenario(os.path.join("assets", selected_scens), int(self.agent_spinbox.get()))

        # solve MAPF
        pibt = PIBT(grid, starts, goals, 42)
        plan = pibt.run(max_timestep=1000)

        print(f"solved: {is_valid_mapf_solution(grid, starts, goals, plan)}")
        print(f"cost: {cost_of_solution(starts,plan)}")

        # save result
        save_configs_for_visualizer(plan, "output.txt")

    def refresh_listboxes(self):
        # Clear both listboxes
        self.map_listbox.delete(0, tk.END)
        self.scen_listbox.delete(0, tk.END)
        
        # Repopulate the listboxes
        self.populate_listboxes()

    def update_scen_selection(self, event):
        # Get selected scen file
        selected_indices = self.scen_listbox.curselection()
        if not selected_indices:
            return

        self.selected_scen = self.scen_listbox.get(selected_indices[0])
        print(f"Selected .scen file: {self.selected_scen}")
    
    def update_scen_listbox(self, event):
        # Get selected map file
        selected_indices = self.map_listbox.curselection()
        if not selected_indices:
            self.map_listbox.select_set(self.selected_map)
            return

        self.selected_map = selected_indices
        # Clear the scen_listbox
        self.scen_listbox.delete(0, tk.END)
        
        selected_map = self.map_listbox.get(selected_indices[0])
        base_name = os.path.splitext(selected_map)[0]
        
        # Populate scen_listbox with matching .scen file
        assets_folder = "assets"
        scen_files = [f for f in os.listdir(assets_folder) if f.endswith(".scen")]
        # Filter and display matching files
        for scen_file in scen_files:
            if base_name in scen_file:
                self.scen_listbox.insert(tk.END, scen_file)
    
    def populate_listboxes(self):
        assets_folder = "assets"
        if not os.path.exists(assets_folder):
            print(f"Folder '{assets_folder}' does not exist.")
            return
        
        map_files = [f for f in os.listdir(assets_folder) if f.endswith(".map")]
        
        # Filter and display matching files
        for map_file in map_files:
            self.map_listbox.insert(tk.END, map_file)

if __name__ == "__main__":
    root = tk.Tk()
    app = FileFilterTool(root)
    root.mainloop()