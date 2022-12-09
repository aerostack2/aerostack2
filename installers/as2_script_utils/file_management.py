import os, shutil, sys

def find_indent_space(file):
  """
  Find the indent space of the file
  """
  indent_levels = set()
  with open(file, 'r') as f:
    for line in f:
      if line.strip().startswith('#') or line.strip() == '':
        continue
      indent_levels.add(len(line) - len(line.lstrip()))
      
      # if line.startswith():
      #   repos.append(line.strip())
  indent_levels.discard(0)
  
  indent_levels = list(indent_levels)
  indent_levels.sort()
  return indent_levels[0]

def extract_repos(repos_file,original_path = ''):
  if original_path != '' and not original_path.endswith('/'):
    original_path = original_path + '/' 
  indent_space = find_indent_space(repos_file)
  repos = []
  with open(repos_file, 'r') as f:
    for line in f:
      if line.strip().startswith('#') or line.strip() == '':
        continue
      if line.startswith(' ' * indent_space) and not line.startswith(' ' * (indent_space + 1)):
        line = original_path + line.strip()[:-1]
        if line.endswith("/"):
          repos.append(line)
        else:
          repos.append(line + "/")
  return repos


