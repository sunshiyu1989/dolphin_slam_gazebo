#!/usr/bin/env python3
"""
Dolphin SLAM - 经验地图单元测试
测试经验地图模块的功能
"""

import pytest
import numpy as np
import sys
import os
import tempfile
import shutil

# 添加项目路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from dolphin_slam.experience_map import ExperienceMap, Experience


class TestExperienceMap:
    """经验地图测试类"""
    
    @pytest.fixture
    def experience_map(self):
        """创建测试用的经验地图"""
        return ExperienceMap(
            match_threshold=0.75,
            lv_factor=0.5,
            pc_factor=0.5,
            min_experience_age=5,
            loop_closure_threshold=0.8
        )
        
    def test_initialization(self, experience_map):
        """测试地图初始化"""
        assert len(experience_map.experiences) == 0
        assert experience_map.current_experience_id is None
        assert experience_map.next_experience_id == 0
        
    def test_create_experience(self, experience_map):
        """测试创建经验"""
        # 创建第一个经验
        exp_id = experience_map.create_experience(
            x=1.0, y=2.0, z=-3.0, theta=0.5,
            visual_template_id=10,
            place_cell_center=(5, 5, 5),
            current_time=100.0
        )
        
        assert exp_id == 0
        assert len(experience_map.experiences) == 1
        assert experience_map.current_experience_id == 0
        
        # 检查经验属性
        exp = experience_map.experiences[exp_id]
        assert exp.x == 1.0
        assert exp.y == 2.0
        assert exp.z == -3.0
        assert exp.theta == 0.5
        assert exp.visual_template_id == 10
        assert exp.place_cell_center == (5, 5, 5)
        
    def test_update_experience(self, experience_map):
        """测试更新经验"""
        # 创建经验
        exp_id = experience_map.create_experience(
            x=0.0, y=0.0, z=0.0, theta=0.0,
            visual_template_id=1,
            place_cell_center=(10, 10, 10),
            current_time=100.0
        )
        
        # 更新位置
        experience_map.update_experience(
            exp_id, x=1.0, y=1.0, z=-1.0, theta=0.1,
            current_time=101.0
        )
        
        exp = experience_map.experiences[exp_id]
        # 位置应该是加权平均
        assert exp.x != 0.0 and exp.x != 1.0
        assert exp.visit_count == 2
        
    def test_find_match(self, experience_map):
        """测试查找匹配经验"""
        # 创建几个经验
        exp1 = experience_map.create_experience(
            0, 0, 0, 0, visual_template_id=1,
            place_cell_center=(10, 10, 10),
            current_time=100.0
        )
        
        exp2 = experience_map.create_experience(
            5, 5, 0, 0, visual_template_id=2,
            place_cell_center=(20, 20, 20),
            current_time=101.0
        )
        
        # 查找匹配 - 相同视觉模板
        match = experience_map.find_match(
            visual_template_id=1,
            visual_similarity=0.9,
            place_cell_center=(11, 11, 11),
            current_position=(0.5, 0.5, 0)
        )
        
        assert match == exp1
        
        # 查找匹配 - 不同视觉模板
        match = experience_map.find_match(
            visual_template_id=3,
            visual_similarity=0.9,
            place_cell_center=(30, 30, 30),
            current_position=(10, 10, 0)
        )
        
        assert match is None
        
    def test_connections(self, experience_map):
        """测试经验之间的连接"""
        # 创建一系列经验
        exp_ids = []
        for i in range(5):
            exp_id = experience_map.create_experience(
                x=float(i), y=0.0, z=0.0, theta=0.0,
                visual_template_id=i,
                place_cell_center=(i*10, 0, 0),
                current_time=100.0 + i
            )
            exp_ids.append(exp_id)
            
        # 检查连接
        # 每个经验应该与前一个连接
        for i in range(1, 5):
            assert exp_ids[i-1] in experience_map.connections[exp_ids[i]]
            assert exp_ids[i] in experience_map.connections[exp_ids[i-1]]
            
    def test_loop_closure_detection(self, experience_map):
        """测试闭环检测"""
        # 创建一个环形路径
        positions = [
            (0, 0), (1, 0), (2, 0), (2, 1),
            (2, 2), (1, 2), (0, 2), (0, 1)
        ]
        
        exp_ids = []
        for i, (x, y) in enumerate(positions):
            exp_id = experience_map.create_experience(
                x=float(x), y=float(y), z=0.0, theta=0.0,
                visual_template_id=i if i < 7 else 0,  # 最后一个与第一个相同
                place_cell_center=(x*10, y*10, 0),
                current_time=100.0 + i
            )
            exp_ids.append(exp_id)
            
        # 检测闭环
        # 最后一个经验应该与第一个形成闭环
        loop_closures = experience_map.detect_loop_closure(exp_ids[-1])
        
        # 应该检测到闭环
        assert len(loop_closures) > 0
        # 闭环应该包含第一个经验
        loop_exp_ids = [lc[0] for lc in loop_closures]
        assert exp_ids[0] in loop_exp_ids
        
    def test_get_path(self, experience_map):
        """测试路径查找"""
        # 创建一个简单的图
        #   1 --- 2
        #   |     |
        #   0 --- 3
        positions = [(0, 0), (0, 1), (1, 1), (1, 0)]
        exp_ids = []
        
        for i, (x, y) in enumerate(positions):
            exp_id = experience_map.create_experience(
                x=float(x), y=float(y), z=0.0, theta=0.0,
                visual_template_id=i,
                place_cell_center=(x*10, y*10, 0),
                current_time=100.0 + i
            )
            exp_ids.append(exp_id)
            
        # 手动添加连接以形成方形
        experience_map._add_connection(exp_ids[0], exp_ids[3])
        
        # 查找路径
        path = experience_map.get_path(exp_ids[0], exp_ids[2])
        
        assert path is not None
        assert len(path) > 0
        assert path[0] == exp_ids[0]
        assert path[-1] == exp_ids[2]
        
    def test_map_bounds(self, experience_map):
        """测试地图边界计算"""
        # 创建分散的经验
        positions = [
            (-5, -3, -2), (10, 8, 1), (2, -7, 3), (0, 0, 0)
        ]
        
        for i, (x, y, z) in enumerate(positions):
            experience_map.create_experience(
                x=float(x), y=float(y), z=float(z), theta=0.0,
                visual_template_id=i,
                place_cell_center=(0, 0, 0),
                current_time=100.0 + i
            )
            
        min_bounds, max_bounds = experience_map.get_map_bounds()
        
        assert np.allclose(min_bounds, [-5, -7, -2])
        assert np.allclose(max_bounds, [10, 8, 3])
        
    def test_save_load_map(self, experience_map):
        """测试地图保存和加载"""
        # 创建一些经验
        for i in range(10):
            experience_map.create_experience(
                x=float(i), y=float(i*2), z=0.0, theta=0.0,
                visual_template_id=i,
                place_cell_center=(i*10, i*10, 0),
                current_time=100.0 + i
            )
            
        # 添加一些闭环
        experience_map.loop_closures_detected = 3
        experience_map.total_distance_traveled = 42.5
        
        # 保存地图
        with tempfile.NamedTemporaryFile(suffix='.pkl', delete=False) as f:
            temp_file = f.name
            
        try:
            experience_map.save_map(temp_file)
            
            # 创建新的地图并加载
            new_map = ExperienceMap()
            new_map.load_map(temp_file)
            
            # 验证加载的内容
            assert len(new_map.experiences) == 10
            assert new_map.loop_closures_detected == 3
            assert new_map.total_distance_traveled == 42.5
            
            # 检查经验内容
            for exp_id in range(10):
                assert exp_id in new_map.experiences
                assert new_map.experiences[exp_id].x == float(exp_id)
                
        finally:
            # 清理临时文件
            if os.path.exists(temp_file):
                os.remove(temp_file)
                
    def test_experience_matching_factors(self, experience_map):
        """测试经验匹配的不同因素"""
        # 创建参考经验
        ref_exp = experience_map.create_experience(
            x=0.0, y=0.0, z=0.0, theta=0.0,
            visual_template_id=1,
            place_cell_center=(10, 10, 10),
            current_time=100.0
        )
        
        # 测试高视觉相似度，低位置细胞相似度
        match = experience_map.find_match(
            visual_template_id=1,
            visual_similarity=0.95,
            place_cell_center=(50, 50, 50),  # 远离
            current_position=(0, 0, 0)
        )
        
        # 由于视觉权重和位置细胞权重相等，可能不匹配
        # 这取决于具体的阈值设置
        
        # 测试低视觉相似度，高位置细胞相似度
        match = experience_map.find_match(
            visual_template_id=2,  # 不同模板
            visual_similarity=0.3,
            place_cell_center=(10, 10, 10),  # 相同
            current_position=(0, 0, 0)
        )
        
        # 不应该匹配（不同的视觉模板）
        assert match is None
        

class TestExperienceMapPerformance:
    """性能测试"""
    
    def test_large_map_performance(self):
        """测试大型地图的性能"""
        import time
        
        experience_map = ExperienceMap()
        
        # 创建大量经验
        start_time = time.time()
        
        for i in range(1000):
            x = np.sin(i * 0.1) * 10
            y = np.cos(i * 0.1) * 10
            z = -5.0
            
            experience_map.create_experience(
                x=x, y=y, z=z, theta=i*0.1,
                visual_template_id=i % 100,  # 100个不同的视觉模板
                place_cell_center=(x, y, 0),
                current_time=100.0 + i * 0.1
            )
            
        creation_time = time.time() - start_time
        print(f"\n创建 1000 个经验用时: {creation_time:.3f} 秒")
        
        # 测试查找性能
        start_time = time.time()
        
        for i in range(100):
            match = experience_map.find_match(
                visual_template_id=i,
                visual_similarity=0.8,
                place_cell_center=(0, 0, 0),
                current_position=(0, 0, 0)
            )
            
        search_time = time.time() - start_time
        print(f"100 次查找用时: {search_time:.3f} 秒")
        
        # 测试路径规划性能
        start_time = time.time()
        
        path = experience_map.get_path(0, 999)
        
        path_time = time.time() - start_time
        print(f"路径规划用时: {path_time:.3f} 秒")
        
        # 性能应该是合理的
        assert creation_time < 1.0  # 创建应该很快
        assert search_time < 1.0    # 查找应该很快
        assert path_time < 5.0      # 路径规划可能较慢
        

class TestExperienceMapEdgeCases:
    """边界情况测试"""
    
    def test_empty_map_operations(self):
        """测试空地图上的操作"""
        experience_map = ExperienceMap()
        
        # 空地图上查找
        match = experience_map.find_match(1, 0.8, (0, 0, 0), (0, 0, 0))
        assert match is None
        
        # 空地图上获取边界
        min_bounds, max_bounds = experience_map.get_map_bounds()
        assert np.array_equal(min_bounds, np.zeros(3))
        assert np.array_equal(max_bounds, np.zeros(3))
        
        # 空地图上获取路径
        path = experience_map.get_path(0, 1)
        assert path is None
        
    def test_single_experience_operations(self):
        """测试只有一个经验的操作"""
        experience_map = ExperienceMap()
        
        exp_id = experience_map.create_experience(
            1, 2, 3, 0, 1, (10, 10, 10), 100.0
        )
        
        # 检测闭环（不应该有）
        loops = experience_map.detect_loop_closure(exp_id)
        assert len(loops) == 0
        
        # 路径（到自己）
        path = experience_map.get_path(exp_id, exp_id)
        assert path == [exp_id]
        
    def test_circular_reference_handling(self):
        """测试循环引用处理"""
        experience_map = ExperienceMap()
        
        # 创建两个相互连接的经验
        exp1 = experience_map.create_experience(
            0, 0, 0, 0, 1, (0, 0, 0), 100.0
        )
        exp2 = experience_map.create_experience(
            1, 0, 0, 0, 2, (10, 0, 0), 101.0
        )
        
        # 应该已经连接
        assert exp2 in experience_map.connections[exp1]
        assert exp1 in experience_map.connections[exp2]
        
        # 再次添加连接不应该重复
        experience_map._add_connection(exp1, exp2)
        assert experience_map.connections[exp1].count(exp2) == 1


if __name__ == '__main__':
    # 运行测试
    pytest.main([__file__, '-v'])
