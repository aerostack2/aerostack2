#!/bin/python3

import os
import sys
import argparse
import re

# INSTALLERS_NAME_REGEX  = "^(.*)install_(.*).bash$"
INSTALLERS_NAME_REGEX  = "^(.*)install_(.*)bash$"

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

def extract_repos(repos_file,original_path):
  if not original_path.endswith('/'):
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

      # if line.startswith():
      #   repos.append(line.strip())
  return repos

def find_installers(repos):
  installers = []  
  for repo in repos:
    files = os.listdir(repo)
    for file in files:
      if re.match(INSTALLERS_NAME_REGEX, file):
        installers.append([repo, file])
  return installers

def run_installers(repos_files):
  installers = find_installers(repos_files)
  for  folder ,installer in installers:
    # files=os.listdir(folder)
    command = f"cd {folder} && bash {installer}"
    os.system(command)

def main():
  #argparse .repos file
  parser = argparse.ArgumentParser(description='Install package dependencies')
  parser.add_argument('-r', '--repos', help='repos file', required=True)
  parser.add_argument('-p', '--aerostack2_home_path', help='aerostack2_home_path', required=True)

  args = parser.parse_args()
  repos = extract_repos(args.repos,args.aerostack2_home_path)
  run_installers(repos)


if __name__ == '__main__':

  main()

