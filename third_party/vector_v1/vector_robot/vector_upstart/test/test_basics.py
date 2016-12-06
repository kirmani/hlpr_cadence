#!/usr/bin/env python

import em
import os
import shutil
import subprocess
import sys
import tempfile
import unittest

import vector_upstart


class TestBasics(unittest.TestCase):

    def setUp(self):
        self.root_dir = tempfile.mkdtemp()

    def tearDown(self):
        shutil.rmtree(self.root_dir)

        # A bit of extra cleanup required which doesn't happen in Interpreter.shutdown. This isn't necessary
        # in the usual course of things where we only do one Job operation per run, but it is required for
        # testing, where we do a bunch of them back to back.
        em.Interpreter._wasProxyInstalled = False

    def pjoin(self, *p):
        return os.path.join(self.root_dir, *p)

    def test_install(self):
        j = vector_upstart.Job(name="foo")
        j.install(sudo=None, root=self.root_dir)

        self.assertTrue(os.path.exists(self.pjoin("usr/sbin/foo-start")), "Start script not created.")
        self.assertTrue(os.path.exists(self.pjoin("usr/sbin/foo-stop")), "Stop script not created.")
        self.assertTrue(os.path.exists(self.pjoin("etc/init/foo.conf")), "Upstart configuration file not created.")

        self.assertEqual(0, subprocess.call(["bash", "-n", self.pjoin("usr/sbin/foo-start")]),
                         "Start script not valid bash syntax.")
        self.assertEqual(0, subprocess.call(["bash", "-n", self.pjoin("usr/sbin/foo-stop")]),
                         "Stop script not valid bash syntax.")

    def test_install_launcher(self):
        j = vector_upstart.Job(name="bar")
        j.add('vector_upstart', 'test/launch/a.launch')
        j.install(sudo=None, root=self.root_dir)

        self.assertTrue(os.path.exists(self.pjoin("etc/ros", os.getenv("ROS_DISTRO"), "bar.d/a.launch")),
                        "Launch file not copied.")
        self.assertFalse(os.path.exists(self.pjoin("etc/ros", os.getenv("ROS_DISTRO"), "bar.d/b.launch")),
                         "Launch copied which shouldn't have been.")

    def test_install_glob(self):
        j = vector_upstart.Job(name="baz")
        j.add('vector_upstart', glob='test/launch/*.launch')
        j.install(sudo=None, root=self.root_dir)

        self.assertTrue(os.path.exists(self.pjoin("etc/ros", os.getenv("ROS_DISTRO"), "baz.d/a.launch")),
                        "Launch file not copied.")
        self.assertTrue(os.path.exists(self.pjoin("etc/ros", os.getenv("ROS_DISTRO"), "baz.d/b.launch")),
                        "Launch file not copied.")

    def test_uninstall(self):
        j = vector_upstart.Job(name="boo")
        j.add('vector_upstart', glob='test/launch/*.launch')
        j.install(sudo=None, root=self.root_dir)
        j.uninstall(sudo=None, root=self.root_dir)

        self.assertFalse(os.path.exists(self.pjoin("etc/ros", os.getenv("ROS_DISTRO"), "boo.d")),
                         "Job dir not removed.")
        self.assertFalse(os.path.exists(self.pjoin("etc/ros", os.getenv("ROS_DISTRO"), "usr/sbin/foo-start")),
                         "Start script not removed.")

    def test_uninstall_user_file(self):
        j = vector_upstart.Job(name="goo")
        j.add('vector_upstart', glob='test/launch/*.launch')
        j.install(sudo=None, root=self.root_dir)
        with open(self.pjoin("etc/ros", os.getenv("ROS_DISTRO"), "goo.d/c.launch"), "w") as f:
            f.write("<launch></launch>")
        j.uninstall(sudo=None, root=self.root_dir)

        self.assertTrue(os.path.exists(self.pjoin("etc/ros", os.getenv("ROS_DISTRO"), "goo.d/c.launch")),
                        "User launch file wrongly removed.")


if __name__ == '__main__':
    import rosunit
    rosunit.unitrun('vector_upstart', 'test_basics', TestBasics)
