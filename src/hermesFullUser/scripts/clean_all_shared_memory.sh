#!/bin/bash

if [ -d /proc/xenomai/registry/native/semaphores/ ]
then
  ./clean_semaphore `find /proc/xenomai/registry/native/semaphores/  -type f -printf '%f\n'`
fi

if [ -d /proc/xenomai/registry/native/heaps/ ]
then
  ./clean_heap `find /proc/xenomai/registry/native/heaps/  -type f -printf '%f\n'`
fi
