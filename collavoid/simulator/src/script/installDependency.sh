#!/usr/bin/env bash

project_dir='../../'

mvn install:install-file -Dfile=${project_dir}lib/simbad/simbad-1.4.jar -DgroupId=simbad -DartifactId=simbad -Dversion=1.4 -Dpackaging=jar -DlocalRepositoryPath=${project_dir}repository

mvn install:install-file -Dfile=${project_dir}lib/java3d/vecmath-1.5.2.jar -DgroupId=java3d -DartifactId=vecmath -Dversion=1.5.2 -Dpackaging=jar -DlocalRepositoryPath=${project_dir}repository