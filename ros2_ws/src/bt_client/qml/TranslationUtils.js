// Translation Management
const translations = {
    "zh": {
        "sequence": " → ",
        "fallback": " ? ",
        "arm": "解鎖",
        "disarm": "上鎖",
        "guided": "導航模式",
        "hold": "停止",
        "registertarget": "註冊目標",
        "navigate": "自主導航",
        "search": "自主搜索",
        "navigatewhilesearch": "自主導航搜索",
        "zigzag": "Z字衝刺",
        "traceeight": "八字機動",
        "stopgimbalcontrol": "停止雲台控制",
        "resumegimbalcontrol": "恢復雲台控制",
        "stopgimbalstreamer": "停止雲台視訊",
        "resumegimbalstreamer": "恢復雲台視訊",
        "takephoto": "拍照",
    },
    "en": {
        "sequence": "→",
        "fallback": "?",
    }
}

function getLabel(name, currentLanguage) {
    let lowerName = name.toLowerCase()
    return translations[currentLanguage][lowerName] || name
}

function switchLanguage(currentLanguage) {
    return currentLanguage === "zh" ? "en" : "zh"
}