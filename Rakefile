require 'rake'
require 'ostruct'

# Project name.
PROJECT = "preform"

# Relevant directories.
DIR = OpenStruct.new
DIR.include = "include"
DIR.src = "src"
DIR.bin = "bin"
DIR.build = "bin/build"
DIR.install = "/usr/local"
DIR.install_include = File.join(DIR.install, "include")

# TODO Use FileList more effectively?
def find_fnames pattern
    fpaths = []
    fpaths += Dir[File.join(DIR.include, "**", "*")]
    fpaths += Dir[File.join(DIR.src, "**", "*")]
    fpaths.select do |fpath|
        fpath =~ pattern if File.file? fpath
    end
end

# Globs.
GLOBS = OpenStruct.new
GLOBS.include_hpp = File.join(DIR.include, "**", "*.hpp")
GLOBS.src_hpp = File.join(DIR.src, "**", "*.hpp")
GLOBS.src_cpp = File.join(DIR.src, "**", "*.cpp")
GLOBS.all_hpp = [GLOBS.include_hpp, GLOBS.src_hpp]
GLOBS.all_cpp = [GLOBS.src_cpp]
GLOBS.all = [*GLOBS.all_hpp, 
             *GLOBS.all_cpp]

# Relevant files.
FILES = OpenStruct.new
FILES.hpp = Rake::FileList.new(*GLOBS.all_hpp)
FILES.cpp = Rake::FileList.new(*GLOBS.all_cpp)
FILES.o = FILES.cpp.pathmap "%{^#{DIR.src},#{DIR.build}}X.o"
FILES.o_debug = FILES.o.pathmap "%X-debug.o"

# Prompt yes/no question.
def yes? str, ans
    ans = ans.downcase[0]
    print str
    print(if ans == "y"
        " [Y/n] "
    else
        " [y/N] "
    end)
    res = STDIN.gets.strip.downcase[0]
    ans = res unless res == nil
    return ans == "y"
end

# Get source filename from object filename.
def get_srcfname_from_objfname objfname
    FILES.cpp.detect do |srcfname|
        srcfname.ext("") ==
        objfname.pathmap("%{^#{DIR.build},#{DIR.src}}X")
                .sub(/-debug$/, "")
    end
end

# Get all include filenames from source filename.
def get_incfnames_from_srcfname srcfname
    reg = /^[ \t]*#include[ \t]*(<[^>]+>|"[^"]+")[ \t]*$/
    arr = []
    dir1 = DIR.include
    dir2 = DIR.src
    File.read(srcfname).each_line do |line|
        if match = line.match(reg)
            if File.exist?(incfname1 = File.join(dir1, match[1][1..-2]))
                arr.push incfname1
                arr.push *get_incfnames_from_srcfname(incfname1)
                next # Include directory takes precedent.
            end
            if File.exist?(incfname2 = File.join(dir2, match[1][1..-2]))
                arr.push incfname2
                arr.push *get_incfnames_from_srcfname(incfname2)
            end
        end
    end
    arr.uniq
end

# Get all include filenames from object filename.
def get_incfnames_from_objfname objfname
    get_incfnames_from_srcfname get_srcfname_from_objfname(objfname)
end

# Get all source filenames from object filename.
def get_srcfnames_from_objfname objfname
    [get_srcfname_from_objfname(objfname),
        *get_incfnames_from_objfname(objfname)]
end

# Get include guard from include filename.
def get_incguard_from_incfname incfname
    incfname.gsub(/[^\w]+/, "_").upcase
end

# Get license comment.
def license_comment
    comment = "/*"
    File.read("LICENSE").each_line do |line|
        comment.concat " "
        comment.concat line
        comment.concat " *"
    end
    comment.concat "/\n"
    comment.concat "/*+-+*/\n" # Boilerplate separator.
end

# Default task.
desc "Default task."
task :default do
    # nothing
end

# Install.
desc "Install files to #{DIR.install}."
task :install do
    sh "mkdir -p #{DIR.install_include}"
    sh "cp -r #{File.join(DIR.include, PROJECT)} #{DIR.install_include}"
end

# Uninstall.
desc "Uninstall files from #{DIR.install}."
task :uninstall do
    sh "rm -r -f #{File.join(DIR.install_include, PROJECT)}"
end

# Doxygen.
namespace :doxygen do

    # Run doxygen.
    desc "Run doxygen."
    task :run do
        sh "doxygen"
    end

    # Clean doxygen output.
    desc "Clean doxygen output (remove docs/doxygen/)."
    task :clean do
        sh "rm -r -f docs/doxygen"
    end

end # namespace :doxygen

# Source management.
namespace :source do

    # Initialize file.
    desc "Initialize file."
    task :init, [:fname] do |task, args|

        # Get filename.
        fname = args.fname
        if File.directory? fname
            raise "#{fname} is a directory"
        end

        # Get source filename.
        dir1 = DIR.include
        dir2 = DIR.src
        srcfname =
        case
        when fname.pathmap("%1d") == dir1 then fname.pathmap("%{^#{dir1}/}p")
        when fname.pathmap("%1d") == dir2 then fname.pathmap("%{^#{dir2}/}p")
        else raise "fname not in known directory"
        end

        # If necessary, make directories.
        mkdir_p fname.pathmap("%d")

        # Is non-existent? or okay to overwrite?
        if not File.exist? fname or 
                      yes? "Overwrite '#{fname}'?", "n"

            # Open file.
            file = File.open fname, "wb"

            # Dump license comment.
            file.write license_comment
            if fname.pathmap("%x") == ".hpp"

                # Initialize with include guard.
                incguard = get_incguard_from_incfname(srcfname)
                file.write <<HPP
\#pragma once
\#ifndef #{incguard}
\#define #{incguard}

\#endif // \#ifndef #{incguard}
HPP
            end
            file.close
        end
    end

    # Generate files from scripts.
    desc "Generate files from scripts."
    task :generate_from_scripts do 
        for fname in find_fnames(/\.(?:hpp|cpp|inl)\.rb$/i)
            # Open file.
            file = File.open(fname.pathmap("%X"), "wb")

            # Dump license comment.
            file.write license_comment

            # Dump warning comment.
            file.write "// A ruby script generates this file, DO NOT EDIT\n\n"

            # Delegate.
            file.write `ruby #{fname}`

            # Close.
            file.close
        end
    end

    desc "Clean whitespace."
    task :clean_whitespace do
        for fname in find_fnames(/\.(?:hpp|cpp)/i)
            # Read file.
            text = File.read(fname)

            # Omit trailing whitespace.
            text.gsub! /[ \t]+$/, ""

            # Wipe file, prepare to rewrite.
            file = File.open(fname, "wb")

            # Write.
            file.write text

            # Close.
            file.close
        end
    end

    # Update license boilerplate.
    desc "Update license boilerplate."
    task :update_license do |task|
        for fname in Rake::FileList.new(*GLOBS.all_hpp)

            # Read file.
            text = File.read fname

            # Wipe file, prepare to rewrite.
            file = File.open fname, "wb"

            # Omit boilerplate.
            if /^\/\*\+-\+\*\/$\n/ =~ text
                text.gsub! /\A.*^\/\*\+-\+\*\/$\n/m, ""
            end

            # Write current license.
            file.write license_comment

            # Write everything back, omitting previous license.
            file.write text

            # Close.
            file.close
        end
    end

    # Count non-blank lines.
    desc "Count non-blank lines, not including boilerplate."
    task :count_lines do
        lines = 0
        lines_total = 0
        for fname in Rake::FileList.new(*GLOBS.all)

            # Read file.
            text = File.read fname

            lines_total += text.lines.count

            # Omit blank lines.
            text.gsub! /^\s*$\n/, ""

            # Omit boilerplate.
            if /^\/\*\+-\+\*\/$\n/ =~ text
                text.gsub! /\A.*^\/\*\+-\+\*\/$\n/m, ""
            end

            # Add lines.
            lines += text.lines.count
        end
        puts "Project has #{lines} non-blank lines, not including boilerplate."
        puts "Project has #{lines_total} lines in total."
    end
end

# Compiler variables.
CC = OpenStruct.new
CC.cc = "clang++"
CC.ccflags = []
CC.ccflags << "-std=c++17"
CC.ccflags << "-Iinclude"
CC.ccflags << "-Wall"
CC.ccflags << "-Wextra"
CC.ccflags << "-march=native"
CC.ccflags << "-mtune=native"
CC.ccflags << "-O2"
CC.ccflags << "-DNDEBUG"
CC.ccflags << "-pthread"
CC.ccflags = CC.ccflags.join " " # To string.

# File rules.
SRC_FNAMES = Rake::FileList["src/**/*.cpp"] 
BIN_FNAMES = SRC_FNAMES.pathmap("%{^src,bin}X")
BIN_FNAMES_DIR = BIN_FNAMES
BIN_FNAMES.size.times do |n|
    src_fname = SRC_FNAMES[n]
    bin_fname = BIN_FNAMES[n]
    file bin_fname => src_fname do
        sh "mkdir -p #{bin_fname.pathmap "%d"}"
        sh "#{CC.cc} #{CC.ccflags} #{src_fname} -o #{bin_fname}"
    end
end

namespace :build do

    # Build examples.
    desc "Build examples."
    task :example => 
    Rake::FileList["src/example/*.cpp"].pathmap("%{^src,bin}X") do
        # nothing
    end

    # Build tests.
    desc "Build tests."
    task :test =>
    Rake::FileList["src/test/*.cpp"].pathmap("%{^src,bin}X") do
        # nothing
    end

end # namespace :build

# Clean.
desc "Clean. (remove bin/)."
task :clean do 
    sh "rm -r -f bin"
end
