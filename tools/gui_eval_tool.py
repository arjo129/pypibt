import os
import tkinter as tk
from tkinter import ttk

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

        # Add a button to print selections in both listboxes
        self.run_button = ttk.Button(root, text="Run", command=self.print_selections)
        self.run_button.grid(row=1, column=0, columnspan=1, pady=10)
      
        self.refresh_button = ttk.Button(root, text="Refresh", command=self.refresh_listboxes)
        self.refresh_button.grid(row=1, column=1, columnspan=1, pady=10)

      
    def print_selections(self):
        # Get selected items from map_listbox
        selected_maps = [self.map_listbox.get(i) for i in self.map_listbox.curselection()]
        # Get selected items from scen_listbox
        selected_scens = [self.scen_listbox.get(i) for i in self.scen_listbox.curselection()]
        
        # Print the selections
        print("Selected .map files:", selected_maps)
        print("Selected .scen files:", selected_scens)


    def refresh_listboxes(self):
        # Clear both listboxes
        self.map_listbox.delete(0, tk.END)
        self.scen_listbox.delete(0, tk.END)
        
        # Repopulate the listboxes
        self.populate_listboxes()
    
    def update_scen_listbox(self, event):
        # Clear the scen_listbox
        self.scen_listbox.delete(0, tk.END)
        
        # Get selected map file
        selected_indices = self.map_listbox.curselection()
        if not selected_indices:
            return
        
        selected_map = self.map_listbox.get(selected_indices[0])
        base_name = os.path.splitext(selected_map)[0]
        
        # Populate scen_listbox with matching .scen file
        assets_folder = "assets"
        matching_scen = f"{base_name}.scen"
        scen_files = [f for f in os.listdir(assets_folder) if f.endswith(".scen")]
        # Filter and display matching files
        for scen_file in scen_files:
            base_name = os.path.splitext(scen_file)[0]
            if base_name in scen_file:
                self.scen_listbox.insert(tk.END, scen_file)
    
    def populate_listboxes(self):
        assets_folder = "assets"
        if not os.path.exists(assets_folder):
            print(f"Folder '{assets_folder}' does not exist.")
            return
        
        map_files = [f for f in os.listdir(assets_folder) if f.endswith(".map")]
        scen_files = [f for f in os.listdir(assets_folder) if f.endswith(".scen")]
        
        map_bases = {os.path.splitext(f)[0] for f in map_files}
        scen_bases = {os.path.splitext(f)[0] for f in scen_files}
        
        # Filter and display matching files
        for map_file in map_files:
            self.map_listbox.insert(tk.END, map_file)

if __name__ == "__main__":
    root = tk.Tk()
    app = FileFilterTool(root)
    root.mainloop()