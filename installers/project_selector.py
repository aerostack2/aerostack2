import os
import re
import tkinter as tk
from shutil import copyfile
import sys
from as2_script_utils.file_management import extract_repos 

projects_selected = []
n_columns = 0
n_rows = 0 

class ScrollableFrame(tk.Frame):
    row = 0
    column = 0
    def __init__(self, master, bg=None,  **kwargs):
        self.bg = bg
        master.grid_columnconfigure(0, weight=1, uniform="fred")
        tk.Frame.__init__(self, master,bg=self.bg, padx=20 , pady= 20, **kwargs)

        # create a canvas object and a vertical scrollbar for scrolling it
        # self.vscrollbar = tk.Scrollbar(self, orient=tk.VERTICAL)
        # self.vscrollbar.pack(side='right', fill="y",  expand=False)

        # self.hscrollbar = tk.Scrollbar(self, orient=tk.HORIZONTAL)
        # self.hscrollbar.pack(side='bottom', fill="x",  expand=False)

        self.canvas = tk.Canvas(self, bg= bg,
                                bd=0,
                                height=600,
                                highlightthickness=0,
                                # yscrollcommand=self.vscrollbar.set,
                                # xscrollcommand=self.hscrollbar.set
                                )
        # self.canvas.pack(side="left", fill="both", expand=True)
        # self.canvas.pack(side="left",fill="both", expand=True) 
        # self.vscrollbar.config(command=self.canvas.yview)
        # self.hscrollbar.config(command=self.canvas.xview)

        # reset the view
        self.canvas.xview_moveto(0)
        self.canvas.yview_moveto(0)

        # create a frame inside the canvas which will be scrolled with it
        self.interior = tk.Frame(self.canvas,bg=bg,**kwargs)
        self.canvas.create_window(0, 0, window=self.interior, anchor="nw")
        self.interior = self
        # self.canvas.pack(anchor='nw',fill='both',expand=True)
        # self.canvas.pack()
        # self.bind('<Configure>', self.set_scrollregion)
        self.pack(fill="both")
    def set_scrollregion(self, event=None):
        """ Set the scroll region on the canvas"""
        self.canvas.configure(scrollregion=self.canvas.bbox('all'))

    def add_checkbutton(self,text,variable):
        # tk.Checkbutton(self.interior,variable=variable, bg = self.bg,text=text,anchor='nw').pack(fill='both')
        tk.Checkbutton(self.interior,variable=variable, bg = self.bg,text=text,anchor='nw').grid(row=self.row,column=self.column,sticky='we')
        self.row +=  1

    def add_text(self,text):
        l = tk.Label(self.interior,bd=5, bg = self.bg,text = text)
        # l.config(font =("Courier", 14))
        l.config(font=("Arial", 12, 'bold'))
        # l.pack(fill='both')
        l.grid(row=self.row,column=self.column,sticky='nswe')
        self.row += 1

    def add_field(self,text):
        print(f'adding field: {text}')
        l = tk.Label(self.interior,bd=5, bg = self.bg,text =text )
        l.config(font =("Arial", 16,'bold','underline'))
        l.grid(row=self.row,column=self.column,sticky='nswe')
        self.row += 1

    def set_row(self,row):
        self.row = row

    def set_column(self,column):
        self.column = column
        self.grid_columnconfigure(column, weight=1, uniform="fred")

    def get_row(self):
        return self.row
    def get_column(self):
        return self.column

def var_states():
    print('projects_selected')
    for project in repos:
        if project['variable'].get():
        # print(f'project {project["name"]} selected in tag {project["tag"]}')
            print(f'project {project["name"]} selected ')

def extract_fields_from_path(path):
    fields = {}
    elems = [ x for x in path.split('/') if x != '']
    fields['container'] = elems[0]
    fields['category'] = elems[1]
    fields['name'] = elems[-1]
    fields['path'] = path
    fields['variable'] = tk.IntVar()
    return fields

def build_repos_graph(repos):
    container_dict = {}
    for repo in repos:
        repo_fields = extract_fields_from_path(repo)
        if repo_fields['container'] not in container_dict:
            container_dict[repo_fields['container']] = {}
        if repo_fields['category'] not in container_dict[repo_fields['container']]:
            container_dict[repo_fields['container']][repo_fields['category']] = []
        container_dict[repo_fields['container']][repo_fields['category']].append(repo_fields)
    # print (container_dict)
    return container_dict 

class MaxRow():
    def __init__(self,row = 0):
        self.row = row
    def get_row(self):
        return self.row
    def set_row(self,row):
        if row > self.row:
            self.row = row

if __name__ == '__main__':
    window = tk.Tk()
    window.title('AS2 Scripts Installer')
    max_row = MaxRow()
    checkbox_pane = ScrollableFrame(window)
    var_list=[]
    # Get submodules list
    repos = extract_repos('projects/core_repositories.repos')
    n_columns=-1
    graph = build_repos_graph(repos)
    for container in graph.keys():
        n_columns += 1
        checkbox_pane.set_column(n_columns)
        checkbox_pane.set_row(0)
        # print(f'container: {container}')
        checkbox_pane.add_field(container)
        for category in graph[container].keys():
            # print(f'category: {category}')
            checkbox_pane.add_text(category)
            for submodule in graph[container][category]:
                # print(submodule)
                checkbox_pane.add_checkbutton(submodule['name'],submodule['variable'])
                max_row.set_row(checkbox_pane.get_row())

    # tk.Button(checkbox_pane.interior,  text='Generate',bd=4, command=var_states).pack()
    tk.Button(checkbox_pane.interior,  text='Generate',bd=4, command=var_states).grid(row=max_row.get_row()+1,column=n_columns // 2,sticky='we')
    tk.Button(checkbox_pane.interior, text='quit', command=window.quit).grid(row=max_row.get_row()+2,column=n_columns // 2 ,sticky='we')
    # window.resizable(False,False)
    window.mainloop()
