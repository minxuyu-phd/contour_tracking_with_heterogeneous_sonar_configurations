#!/usr/bin/env python3
"""
Generate PDF report for Stonefish underwater robot simulation framework analysis.
"""

from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import getSampleStyleSheet, ParagraphStyle
from reportlab.lib.units import cm, mm
from reportlab.lib.colors import HexColor, black, darkblue
from reportlab.platypus import (
    SimpleDocTemplate, Paragraph, Spacer, Table, TableStyle,
    PageBreak, Preformatted, KeepTogether
)
from reportlab.lib.enums import TA_LEFT, TA_CENTER, TA_JUSTIFY
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
import os

# Try to register Chinese fonts
def register_chinese_fonts():
    """Try to register Chinese fonts for proper rendering."""
    font_paths = [
        '/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc',
        '/usr/share/fonts/truetype/wqy/wqy-microhei.ttc',
        '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',
        '/usr/share/fonts/truetype/noto/NotoSansCJK-Regular.ttc',
        '/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf',
        '/usr/share/fonts/truetype/arphic/uming.ttc',
    ]

    for font_path in font_paths:
        if os.path.exists(font_path):
            try:
                pdfmetrics.registerFont(TTFont('Chinese', font_path))
                return 'Chinese'
            except:
                continue
    return None

chinese_font = register_chinese_fonts()

def create_styles():
    """Create custom styles for the document."""
    styles = getSampleStyleSheet()

    base_font = chinese_font if chinese_font else 'Helvetica'
    base_font_bold = chinese_font if chinese_font else 'Helvetica-Bold'

    # Title style
    styles.add(ParagraphStyle(
        name='CustomTitle',
        fontName=base_font_bold,
        fontSize=20,
        leading=24,
        alignment=TA_CENTER,
        spaceAfter=20,
        textColor=darkblue
    ))

    # Heading 1
    styles.add(ParagraphStyle(
        name='Heading1Custom',
        fontName=base_font_bold,
        fontSize=16,
        leading=20,
        spaceBefore=20,
        spaceAfter=10,
        textColor=HexColor('#1a5276')
    ))

    # Heading 2
    styles.add(ParagraphStyle(
        name='Heading2Custom',
        fontName=base_font_bold,
        fontSize=13,
        leading=16,
        spaceBefore=15,
        spaceAfter=8,
        textColor=HexColor('#2874a6')
    ))

    # Heading 3
    styles.add(ParagraphStyle(
        name='Heading3Custom',
        fontName=base_font_bold,
        fontSize=11,
        leading=14,
        spaceBefore=10,
        spaceAfter=6,
        textColor=HexColor('#3498db')
    ))

    # Body text
    styles.add(ParagraphStyle(
        name='BodyCustom',
        fontName=base_font,
        fontSize=10,
        leading=14,
        alignment=TA_JUSTIFY,
        spaceAfter=8
    ))

    # Code style
    styles.add(ParagraphStyle(
        name='CodeCustom',
        fontName='Courier',
        fontSize=8,
        leading=10,
        leftIndent=10,
        spaceAfter=8,
        backColor=HexColor('#f4f4f4')
    ))

    # Bullet style
    styles.add(ParagraphStyle(
        name='BulletCustom',
        fontName=base_font,
        fontSize=10,
        leading=13,
        leftIndent=20,
        bulletIndent=10,
        spaceAfter=4
    ))

    return styles

def create_document():
    """Create the PDF document."""
    output_path = '/home/bsa/2026/stonefish/docs/Stonefish_Analysis_Report.pdf'

    doc = SimpleDocTemplate(
        output_path,
        pagesize=A4,
        rightMargin=2*cm,
        leftMargin=2*cm,
        topMargin=2*cm,
        bottomMargin=2*cm
    )

    styles = create_styles()
    story = []

    # Title
    story.append(Paragraph("Stonefish 水下机器人仿真框架分析报告", styles['CustomTitle']))
    story.append(Spacer(1, 10))
    story.append(Paragraph("Stonefish Underwater Robot Simulation Framework Analysis", styles['BodyCustom']))
    story.append(Spacer(1, 20))

    # Overview
    story.append(Paragraph("概述", styles['Heading1Custom']))
    story.append(Paragraph(
        "Stonefish 是一个专为海洋机器人研究开发的高级仿真工具。它基于 Bullet Physics 物理引擎，"
        "并扩展了海洋机器人所需的水动力学计算。该框架包含自研的渲染管线，能够实现逼真的水下光学效果"
        "（考虑波长相关的光吸收和散射）。",
        styles['BodyCustom']
    ))

    # Section 1: Architecture
    story.append(Paragraph("一、仿真器架构与用户程序集成", styles['Heading1Custom']))

    story.append(Paragraph("1.1 核心架构", styles['Heading2Custom']))
    story.append(Paragraph(
        "Stonefish采用双层架构设计：",
        styles['BodyCustom']
    ))

    arch_data = [
        ['层级', '组件', '功能'],
        ['应用层', 'SimulationApp', '应用生命周期管理、仿真循环'],
        ['', 'GraphicalSimulationApp', '带GUI渲染的图形化应用'],
        ['', 'ConsoleSimulationApp', '无头仿真（无渲染）'],
        ['核心层', 'SimulationManager', '物理世界管理（Bullet Physics）'],
        ['', '', '水动力学计算'],
        ['', '', '实体/机器人/传感器/执行器管理'],
    ]

    arch_table = Table(arch_data, colWidths=[3*cm, 5*cm, 7*cm])
    arch_table.setStyle(TableStyle([
        ('BACKGROUND', (0, 0), (-1, 0), HexColor('#2874a6')),
        ('TEXTCOLOR', (0, 0), (-1, 0), HexColor('#ffffff')),
        ('ALIGN', (0, 0), (-1, -1), 'LEFT'),
        ('FONTNAME', (0, 0), (-1, 0), 'Helvetica-Bold'),
        ('FONTSIZE', (0, 0), (-1, -1), 9),
        ('BOTTOMPADDING', (0, 0), (-1, 0), 8),
        ('BACKGROUND', (0, 1), (-1, -1), HexColor('#f8f9fa')),
        ('GRID', (0, 0), (-1, -1), 0.5, HexColor('#dee2e6')),
        ('VALIGN', (0, 0), (-1, -1), 'MIDDLE'),
        ('LEFTPADDING', (0, 0), (-1, -1), 6),
        ('RIGHTPADDING', (0, 0), (-1, -1), 6),
    ]))
    story.append(arch_table)
    story.append(Spacer(1, 10))

    story.append(Paragraph("关键文件位置：", styles['Heading3Custom']))
    files_list = [
        "• SimulationManager.h - 仿真管理器核心",
        "• SimulationApp.h - 应用基类",
        "• GraphicalSimulationApp.h - 图形化应用",
        "• Robot.h - 机器人定义",
    ]
    for item in files_list:
        story.append(Paragraph(item, styles['BulletCustom']))

    story.append(Paragraph("1.2 用户程序集成方式", styles['Heading2Custom']))

    story.append(Paragraph("方式A：C++代码直接集成", styles['Heading3Custom']))
    story.append(Paragraph(
        "用户需要：1) 继承 SimulationManager，重写 BuildScenario() 方法；"
        "2) 可选继承 GraphicalSimulationApp 自定义GUI；"
        "3) 在 main() 中实例化并运行。",
        styles['BodyCustom']
    ))

    code_example1 = """class MySimManager : public sf::SimulationManager {
public:
    void BuildScenario() override {
        CreateMaterial("Aluminium", 2710.0, 0.7);
        EnableOcean(0.0, Fluid());
        Robot* auv = new Robot("MyAUV", false);
        AddRobot(auv, Transform::getIdentity());
    }
    void SimulationStepCompleted(Scalar dt) override {
        // 读取传感器/控制执行器
    }
};"""
    story.append(Preformatted(code_example1, styles['CodeCustom']))

    story.append(Paragraph("方式B：XML场景文件配置", styles['Heading3Custom']))
    story.append(Paragraph(
        "使用 ScenarioParser 解析XML场景描述文件(.scn)。",
        styles['BodyCustom']
    ))

    story.append(Paragraph("1.3 ROS集成", styles['Heading2Custom']))
    story.append(Paragraph(
        "通过 stonefish_ros 包可与ROS架构无缝集成，实现标准的仿真节点。"
        "GitHub: https://github.com/patrykcieslak/stonefish_ros",
        styles['BodyCustom']
    ))

    # Section 2: Terrain
    story.append(PageBreak())
    story.append(Paragraph("二、水底地形配置", styles['Heading1Custom']))

    story.append(Paragraph("2.1 Terrain 类（高度图地形）", styles['Heading2Custom']))
    story.append(Paragraph(
        "从灰度高度图图像创建海底地形，支持8位或16位PNG图像。",
        styles['BodyCustom']
    ))

    terrain_params = [
        ['参数', '类型', '说明'],
        ['uniqueName', 'string', '地形名称'],
        ['pathToHeightmap', 'string', '高度图PNG文件路径'],
        ['scaleX', 'Scalar', 'X方向比例 [m/像素]'],
        ['scaleY', 'Scalar', 'Y方向比例 [m/像素]'],
        ['height', 'Scalar', '最大高度 [m]'],
        ['material', 'string', '物理材料名称'],
        ['look', 'string', '渲染外观名称'],
    ]

    terrain_table = Table(terrain_params, colWidths=[3.5*cm, 2.5*cm, 9*cm])
    terrain_table.setStyle(TableStyle([
        ('BACKGROUND', (0, 0), (-1, 0), HexColor('#2874a6')),
        ('TEXTCOLOR', (0, 0), (-1, 0), HexColor('#ffffff')),
        ('ALIGN', (0, 0), (-1, -1), 'LEFT'),
        ('FONTNAME', (0, 0), (-1, 0), 'Helvetica-Bold'),
        ('FONTSIZE', (0, 0), (-1, -1), 9),
        ('BOTTOMPADDING', (0, 0), (-1, 0), 8),
        ('BACKGROUND', (0, 1), (-1, -1), HexColor('#f8f9fa')),
        ('GRID', (0, 0), (-1, -1), 0.5, HexColor('#dee2e6')),
    ]))
    story.append(terrain_table)
    story.append(Spacer(1, 10))

    story.append(Paragraph("2.2 海洋环境配置", styles['Heading2Custom']))

    ocean_xml = """<environment>
    <ned latitude="40.0" longitude="3.0"/>
    <ocean>
        <water density="1025.0" jerlov="0.25"/>
        <waves height="0.5"/>
        <current type="uniform">
            <velocity xyz="0.5 0.0 0.0"/>
        </current>
    </ocean>
</environment>"""
    story.append(Preformatted(ocean_xml, styles['CodeCustom']))

    ocean_params = [
        ['参数', '说明'],
        ['density', '水密度 [kg/m³]，默认约1000'],
        ['jerlov', '水的光学清澈度(0-1)，Jerlov水体类型'],
        ['waves height', '波浪高度，0表示无波浪'],
        ['current type', '海流类型：uniform（均匀）或 jet（射流）'],
    ]

    ocean_table = Table(ocean_params, colWidths=[4*cm, 11*cm])
    ocean_table.setStyle(TableStyle([
        ('BACKGROUND', (0, 0), (-1, 0), HexColor('#2874a6')),
        ('TEXTCOLOR', (0, 0), (-1, 0), HexColor('#ffffff')),
        ('ALIGN', (0, 0), (-1, -1), 'LEFT'),
        ('FONTNAME', (0, 0), (-1, 0), 'Helvetica-Bold'),
        ('FONTSIZE', (0, 0), (-1, -1), 9),
        ('BACKGROUND', (0, 1), (-1, -1), HexColor('#f8f9fa')),
        ('GRID', (0, 0), (-1, -1), 0.5, HexColor('#dee2e6')),
    ]))
    story.append(ocean_table)

    # Section 3: AUV Dynamics
    story.append(PageBreak())
    story.append(Paragraph("三、AUV动力学与操纵", styles['Heading1Custom']))

    story.append(Paragraph("3.1 水动力学模型", styles['Heading2Custom']))
    story.append(Paragraph(
        "Stonefish基于实际几何形状计算水动力，包括三种主要力：",
        styles['BodyCustom']
    ))

    hydro_forces = [
        ['力类型', '描述', '特性'],
        ['浮力 (Buoyancy)', '基于排水体积和流体密度', '考虑浮心偏移'],
        ['形状阻力 (Form Drag)', '二次阻力系数 Cd', '与速度平方成正比'],
        ['表面摩擦 (Skin Friction)', '粘性阻力系数 Cf', '线性阻尼分量'],
    ]

    hydro_table = Table(hydro_forces, colWidths=[4*cm, 5.5*cm, 5.5*cm])
    hydro_table.setStyle(TableStyle([
        ('BACKGROUND', (0, 0), (-1, 0), HexColor('#2874a6')),
        ('TEXTCOLOR', (0, 0), (-1, 0), HexColor('#ffffff')),
        ('ALIGN', (0, 0), (-1, -1), 'LEFT'),
        ('FONTNAME', (0, 0), (-1, 0), 'Helvetica-Bold'),
        ('FONTSIZE', (0, 0), (-1, -1), 9),
        ('BACKGROUND', (0, 1), (-1, -1), HexColor('#f8f9fa')),
        ('GRID', (0, 0), (-1, -1), 0.5, HexColor('#dee2e6')),
    ]))
    story.append(hydro_table)
    story.append(Spacer(1, 10))

    story.append(Paragraph("3.2 推进器配置", styles['Heading2Custom']))
    story.append(Paragraph(
        "Thruster 类提供高级推进器仿真，支持多种动力学模型。",
        styles['BodyCustom']
    ))

    story.append(Paragraph("转子动力学模型类型：", styles['Heading3Custom']))
    rotor_models = [
        ['模型', '描述'],
        ['zero_order', '直接透传'],
        ['first_order', '一阶滞后 (参数: tau)'],
        ['yoerger', '非线性二次阻尼模型'],
        ['bessa', '电机电气模型'],
        ['mechanical_pi', '带PI控制器的机械轴'],
    ]

    rotor_table = Table(rotor_models, colWidths=[4*cm, 11*cm])
    rotor_table.setStyle(TableStyle([
        ('BACKGROUND', (0, 0), (-1, 0), HexColor('#27ae60')),
        ('TEXTCOLOR', (0, 0), (-1, 0), HexColor('#ffffff')),
        ('ALIGN', (0, 0), (-1, -1), 'LEFT'),
        ('FONTNAME', (0, 0), (-1, 0), 'Helvetica-Bold'),
        ('FONTSIZE', (0, 0), (-1, -1), 9),
        ('BACKGROUND', (0, 1), (-1, -1), HexColor('#f8f9fa')),
        ('GRID', (0, 0), (-1, -1), 0.5, HexColor('#dee2e6')),
    ]))
    story.append(rotor_table)
    story.append(Spacer(1, 10))

    story.append(Paragraph("推力模型类型：", styles['Heading3Custom']))
    thrust_models = [
        ['模型', '描述'],
        ['quadratic', '简单二次模型 T = kt * w * |w|'],
        ['deadband', '带死区的非对称模型'],
        ['interpolated', '查表线性插值'],
        ['fluid_dynamics', '流体动力学模型（最真实）'],
    ]

    thrust_table = Table(thrust_models, colWidths=[4*cm, 11*cm])
    thrust_table.setStyle(TableStyle([
        ('BACKGROUND', (0, 0), (-1, 0), HexColor('#e67e22')),
        ('TEXTCOLOR', (0, 0), (-1, 0), HexColor('#ffffff')),
        ('ALIGN', (0, 0), (-1, -1), 'LEFT'),
        ('FONTNAME', (0, 0), (-1, 0), 'Helvetica-Bold'),
        ('FONTSIZE', (0, 0), (-1, -1), 9),
        ('BACKGROUND', (0, 1), (-1, -1), HexColor('#f8f9fa')),
        ('GRID', (0, 0), (-1, -1), 0.5, HexColor('#dee2e6')),
    ]))
    story.append(thrust_table)
    story.append(Spacer(1, 10))

    story.append(Paragraph("3.3 控制AUV示例", styles['Heading2Custom']))
    control_code = """void SimulationStepCompleted(Scalar dt) override {
    Thruster* th1 = dynamic_cast<Thruster*>(
        getRobot("GIRONA500")->getActuator("ThrusterSurgePort"));
    Thruster* th2 = dynamic_cast<Thruster*>(
        getRobot("GIRONA500")->getActuator("ThrusterSurgeStarboard"));

    Scalar surge_cmd = 0.5;  // 归一化: -1 到 1
    Scalar yaw_cmd = 0.1;
    th1->setSetpoint(-surge_cmd - yaw_cmd);
    th2->setSetpoint(-surge_cmd + yaw_cmd);
}"""
    story.append(Preformatted(control_code, styles['CodeCustom']))

    # Section 4: Sonar Sensors
    story.append(PageBreak())
    story.append(Paragraph("四、声纳传感器使用", styles['Heading1Custom']))

    story.append(Paragraph("4.1 声纳类型概览", styles['Heading2Custom']))

    sonar_types = [
        ['类型', '类名', '描述'],
        ['MSIS', 'sf::MSIS', '机械扫描成像声纳（单波束旋转）'],
        ['FLS', 'sf::FLS', '前视声纳（多波束固定）'],
        ['SSS', 'sf::SSS', '侧扫声纳（瀑布图显示）'],
        ['Multibeam2', 'sf::Multibeam2', '多波束声纳（基于深度相机）'],
    ]

    sonar_table = Table(sonar_types, colWidths=[3*cm, 3*cm, 9*cm])
    sonar_table.setStyle(TableStyle([
        ('BACKGROUND', (0, 0), (-1, 0), HexColor('#8e44ad')),
        ('TEXTCOLOR', (0, 0), (-1, 0), HexColor('#ffffff')),
        ('ALIGN', (0, 0), (-1, -1), 'LEFT'),
        ('FONTNAME', (0, 0), (-1, 0), 'Helvetica-Bold'),
        ('FONTSIZE', (0, 0), (-1, -1), 9),
        ('BACKGROUND', (0, 1), (-1, -1), HexColor('#f8f9fa')),
        ('GRID', (0, 0), (-1, -1), 0.5, HexColor('#dee2e6')),
    ]))
    story.append(sonar_table)
    story.append(Spacer(1, 10))

    story.append(Paragraph("4.2 MSIS 单波束机械扫描声纳", styles['Heading2Custom']))
    story.append(Paragraph(
        "MSIS模拟一个机械旋转的单波束声纳，逐步扫描形成扇形图像。",
        styles['BodyCustom']
    ))

    story.append(Paragraph("构造函数参数：", styles['Heading3Custom']))
    msis_params = [
        ['参数', '类型', '说明'],
        ['uniqueName', 'string', '传感器名称'],
        ['stepAngleDeg', 'Scalar', '旋转步进角度 [度]'],
        ['numOfBins', 'unsigned int', '距离分辨率（bin数量）'],
        ['horizontalBeamWidthDeg', 'Scalar', '水平波束宽度 [度]'],
        ['verticalBeamWidthDeg', 'Scalar', '垂直波束宽度 [度]'],
        ['minRotationDeg', 'Scalar', '最小旋转角度 [度]'],
        ['maxRotationDeg', 'Scalar', '最大旋转角度 [度]'],
        ['minRange', 'Scalar', '最小测量距离 [m]'],
        ['maxRange', 'Scalar', '最大测量距离 [m]'],
        ['cm', 'ColorMap', '显示用颜色图'],
        ['outputFormat', 'SonarOutputFormat', '输出数据格式'],
        ['frequency', 'Scalar', '采样频率 [Hz]，-1表示自动'],
    ]

    msis_table = Table(msis_params, colWidths=[4.5*cm, 3*cm, 7.5*cm])
    msis_table.setStyle(TableStyle([
        ('BACKGROUND', (0, 0), (-1, 0), HexColor('#2874a6')),
        ('TEXTCOLOR', (0, 0), (-1, 0), HexColor('#ffffff')),
        ('ALIGN', (0, 0), (-1, -1), 'LEFT'),
        ('FONTNAME', (0, 0), (-1, 0), 'Helvetica-Bold'),
        ('FONTSIZE', (0, 0), (-1, -1), 8),
        ('BACKGROUND', (0, 1), (-1, -1), HexColor('#f8f9fa')),
        ('GRID', (0, 0), (-1, -1), 0.5, HexColor('#dee2e6')),
    ]))
    story.append(msis_table)
    story.append(Spacer(1, 10))

    story.append(Paragraph("XML配置示例：", styles['Heading3Custom']))
    msis_xml = """<sensor name="msis" type="msis" rate="30.0">
    <link name="Vehicle"/>
    <origin xyz="0.5 0.0 0.2" rpy="0.0 0.0 0.0"/>
    <specs step="1.8" bins="500"
           horizontal_beam_width="3.0" vertical_beam_width="35.0"
           output_format="uint8"/>
    <settings range_min="0.5" range_max="30.0"
              rotation_min="-180.0" rotation_max="180.0" gain="1.2"/>
    <noise multiplicative="0.02" additive="0.04"/>
    <display colormap="hot"/>
</sensor>"""
    story.append(Preformatted(msis_xml, styles['CodeCustom']))

    story.append(Paragraph("C++中使用MSIS：", styles['Heading3Custom']))
    msis_code = """MSIS* msis = dynamic_cast<MSIS*>(getSensor("msis"));

// 安装数据回调
msis->InstallNewDataHandler([](MSIS* s) {
    void* data = s->getImageDataPointer();
    unsigned int width, height;
    s->getDisplayResolution(width, height);
    int currentStep = s->getCurrentRotationStep();
});

// 动态调整参数
msis->setRangeMin(1.0);
msis->setRangeMax(50.0);
msis->setGain(1.5);
msis->setRotationLimits(-90.0, 90.0);"""
    story.append(Preformatted(msis_code, styles['CodeCustom']))

    story.append(Paragraph("4.3 数据输出格式", styles['Heading2Custom']))
    output_formats = [
        ['格式', '类型', '范围'],
        ['U8', 'uint8', '0-255'],
        ['U16', 'uint16', '0-65535'],
        ['U32', 'uint32', '0-4294967295'],
        ['F32', 'float32', '浮点数'],
    ]

    format_table = Table(output_formats, colWidths=[3*cm, 4*cm, 8*cm])
    format_table.setStyle(TableStyle([
        ('BACKGROUND', (0, 0), (-1, 0), HexColor('#16a085')),
        ('TEXTCOLOR', (0, 0), (-1, 0), HexColor('#ffffff')),
        ('ALIGN', (0, 0), (-1, -1), 'LEFT'),
        ('FONTNAME', (0, 0), (-1, 0), 'Helvetica-Bold'),
        ('FONTSIZE', (0, 0), (-1, -1), 9),
        ('BACKGROUND', (0, 1), (-1, -1), HexColor('#f8f9fa')),
        ('GRID', (0, 0), (-1, -1), 0.5, HexColor('#dee2e6')),
    ]))
    story.append(format_table)
    story.append(Spacer(1, 10))

    story.append(Paragraph("4.4 颜色图选项", styles['Heading2Custom']))
    colormaps = [
        ['颜色图', '描述'],
        ['HOT', '红-黄-白'],
        ['JET', '蓝-青-绿-黄-红'],
        ['PERULA', '蓝-绿-黄-红'],
        ['GREEN_BLUE', '绿-蓝（默认）'],
        ['ORANGE_COPPER', '橙-铜'],
        ['COLD_BLUE', '蓝-青'],
        ['GREY', '灰度'],
    ]

    colormap_table = Table(colormaps, colWidths=[4*cm, 11*cm])
    colormap_table.setStyle(TableStyle([
        ('BACKGROUND', (0, 0), (-1, 0), HexColor('#c0392b')),
        ('TEXTCOLOR', (0, 0), (-1, 0), HexColor('#ffffff')),
        ('ALIGN', (0, 0), (-1, -1), 'LEFT'),
        ('FONTNAME', (0, 0), (-1, 0), 'Helvetica-Bold'),
        ('FONTSIZE', (0, 0), (-1, -1), 9),
        ('BACKGROUND', (0, 1), (-1, -1), HexColor('#f8f9fa')),
        ('GRID', (0, 0), (-1, -1), 0.5, HexColor('#dee2e6')),
    ]))
    story.append(colormap_table)

    # Section 5: File Paths
    story.append(PageBreak())
    story.append(Paragraph("五、关键文件路径汇总", styles['Heading1Custom']))

    file_paths = [
        ['组件', '路径'],
        ['核心头文件', 'Library/include/core/'],
        ['实体类型', 'Library/include/entities/'],
        ['传感器', 'Library/include/sensors/'],
        ['执行器', 'Library/include/actuators/'],
        ['关节', 'Library/include/joints/'],
        ['图形', 'Library/include/graphics/'],
        ['示例场景', 'Tests/Data/*.scn'],
        ['文档', 'docs/ (Sphinx格式)'],
    ]

    path_table = Table(file_paths, colWidths=[4*cm, 11*cm])
    path_table.setStyle(TableStyle([
        ('BACKGROUND', (0, 0), (-1, 0), HexColor('#34495e')),
        ('TEXTCOLOR', (0, 0), (-1, 0), HexColor('#ffffff')),
        ('ALIGN', (0, 0), (-1, -1), 'LEFT'),
        ('FONTNAME', (0, 0), (-1, 0), 'Helvetica-Bold'),
        ('FONTSIZE', (0, 0), (-1, -1), 9),
        ('BACKGROUND', (0, 1), (-1, -1), HexColor('#f8f9fa')),
        ('GRID', (0, 0), (-1, -1), 0.5, HexColor('#dee2e6')),
    ]))
    story.append(path_table)

    # Summary
    story.append(Spacer(1, 20))
    story.append(Paragraph("总结", styles['Heading1Custom']))
    story.append(Paragraph(
        "Stonefish提供了一个完整的水下机器人仿真环境，支持：",
        styles['BodyCustom']
    ))

    summary_items = [
        "• 灵活的架构：可通过C++代码或XML场景文件配置",
        "• 真实的物理：基于几何的水动力学计算",
        "• 丰富的传感器：多种声纳类型（MSIS、FLS、SSS）、相机、DVL等",
        "• 精确的推进：多种转子动力学和推力模型",
        "• ROS集成：通过stonefish_ros包实现",
    ]
    for item in summary_items:
        story.append(Paragraph(item, styles['BulletCustom']))

    story.append(Spacer(1, 15))
    story.append(Paragraph(
        "对于MSIS单波束机械扫描声纳的使用，关键是理解其扫描机制（步进旋转）、"
        "输出格式选择，以及如何通过回调函数处理实时数据。",
        styles['BodyCustom']
    ))

    # Build PDF
    doc.build(story)
    print(f"PDF generated successfully: {output_path}")
    return output_path

if __name__ == '__main__':
    create_document()
