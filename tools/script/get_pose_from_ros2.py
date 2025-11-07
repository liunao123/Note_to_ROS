#!/usr/bin/env python3
import argparse
import glob
import os
import sqlite3
import sys
from dataclasses import dataclass
from typing import List


def try_import_rosbag2_py():
    try:
        import rosbag2_py  # type: ignore
        return rosbag2_py
    except Exception:
        return None


@dataclass
class TopicInfo:
    topic_id: int
    name: str
    type_str: str


def list_db3_files(bag_path: str) -> List[str]:
    if os.path.isdir(bag_path):
        dbs = sorted(glob.glob(os.path.join(bag_path, "*.db3")))
        return dbs
    if os.path.isfile(bag_path) and bag_path.endswith(".db3"):
        return [bag_path]
    return []


def read_topics_sqlite(db3_path: str) -> List[TopicInfo]:
    connection = sqlite3.connect(db3_path)
    try:
        cursor = connection.cursor()
        cursor.execute("SELECT id, name, type FROM topics")
        rows = cursor.fetchall()
        topics: List[TopicInfo] = []
        for row in rows:
            topics.append(TopicInfo(topic_id=int(row[0]), name=str(row[1]), type_str=str(row[2])))
        return topics
    finally:
        connection.close()


def list_all_topics_sqlite(db3_files: List[str]) -> List[TopicInfo]:
    topics_by_name = {}
    for db in db3_files:
        for t in read_topics_sqlite(db):
            topics_by_name[t.name] = t
    return list(topics_by_name.values())


def list_all_topics_rosbag2_py(bag_path: str) -> List[TopicInfo]:
    rosbag2_py = try_import_rosbag2_py()
    if rosbag2_py is None:
        raise RuntimeError("rosbag2_py not available. Please source your ROS 2 environment.")

    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id="sqlite3")
    converter_options = rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr")
    reader.open(storage_options, converter_options)

    topics_meta = reader.get_all_topics_and_types()
    topics: List[TopicInfo] = []
    for meta in topics_meta:
        topics.append(TopicInfo(topic_id=-1, name=meta.name, type_str=meta.type))
    return topics


def main() -> None:
    parser = argparse.ArgumentParser(description="List all topics in a ROS 2 bag (rosbag2)")
    parser.add_argument("bag", help="Path to rosbag2 directory or .db3 file")
    parser.add_argument("--prefer", choices=["sqlite", "rosbag2_py"], default="sqlite", help="Backend preference")
    args = parser.parse_args()

    bag_path = os.path.abspath(args.bag)
    if not os.path.exists(bag_path):
        print(f"Bag path does not exist: {bag_path}", file=sys.stderr)
        sys.exit(1)

    try:
        if args.prefer == "sqlite":
            db3_files = list_db3_files(bag_path)
            if not db3_files and os.path.isdir(bag_path):
                topics = list_all_topics_rosbag2_py(bag_path)
            else:
                topics = list_all_topics_sqlite(db3_files)
        else:
            topics = list_all_topics_rosbag2_py(bag_path)
    except Exception:
        # fallback
        if args.prefer == "sqlite":
            topics = list_all_topics_rosbag2_py(bag_path)
        else:
            db3_files = list_db3_files(bag_path)
            topics = list_all_topics_sqlite(db3_files)

    if not topics:
        print("No topics found.")
        return

    print("Topics:")
    for t in topics:
        print(f"- {t.name} [{t.type_str}]")


if __name__ == "__main__":
    main()


