// SPDX-License-Identifier: Apache-2.0
// LD_PRELOAD probe: whenever SMART_ACCESS_CFG_FILE_PATH is set, appends a line with
// the names of the process's threads ('|'-separated) to SMARTMICRO_SETENV_PROBE_LOG.
#include <dirent.h>
#include <dlfcn.h>
#include <fcntl.h>
#include <unistd.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>

extern "C" int setenv(const char * name, const char * value, int overwrite)
{
  using Setenv = int (*)(const char *, const char *, int);
  static const auto real = reinterpret_cast<Setenv>(dlsym(RTLD_NEXT, "setenv"));
  const char * log = std::getenv("SMARTMICRO_SETENV_PROBE_LOG");
  if (log && std::strcmp(name, "SMART_ACCESS_CFG_FILE_PATH") == 0) {
    // One line: the names of the process's threads, e.g. "smartmicro_rada".
    char line[4096];
    size_t length = 0;
    if (DIR * tasks = opendir("/proc/self/task")) {
      while (const dirent * entry = readdir(tasks)) {
        if (entry->d_name[0] == '.') {continue;}
        char path[300];
        std::snprintf(path, sizeof(path), "/proc/self/task/%s/comm", entry->d_name);
        const int comm = open(path, O_RDONLY);
        if (comm < 0) {continue;}
        char name_buffer[32];
        const auto count = read(comm, name_buffer, sizeof(name_buffer) - 1);
        close(comm);
        for (ssize_t i = 0; i < count && length + 2 < sizeof(line); ++i) {
          line[length++] = name_buffer[i] == '\n' ? '|' : name_buffer[i];
        }
      }
      closedir(tasks);
    }
    line[length++] = '\n';
    const int fd = open(log, O_WRONLY | O_CREAT | O_APPEND, 0600);
    if (fd >= 0) {
      [[maybe_unused]] const auto written = write(fd, line, length);
      close(fd);
    }
  }
  return real(name, value, overwrite);
}
