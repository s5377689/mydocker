// // Node Size Calculations
// function calculateNodeWidth(
//     btNode, nodeTypeIconWidth,
//     text, fontSize, fontFamily, fontBold,
//     nodeWidthMargin, showAttributes)
// {
//     let nodeTextWidth = calculateTextWidth(text, fontSize, fontFamily, fontBold)
//     let attributeWidth = 0

//     if (showAttributes && btNode.attributes && Object.keys(btNode.attributes).length > 0) {
//         let maxAttrWidth = 0
//         for (let key in btNode.attributes) {
//             let attrLineWidth = (key.length + btNode.attributes[key].length + 2) * 7 + 22
//             maxAttrWidth = Math.max(maxAttrWidth, attrLineWidth)
//         }
//         attributeWidth = maxAttrWidth
//     }

//     return Math.max(
//         nodeTypeIconWidth + nodeTextWidth + nodeWidthMargin,
//         showAttributes ?  attributeWidth + nodeWidthMargin : 0
//     )
// }

// function calculateTextWidth(text, pointSize, fontFamily, bold) {
//     // Create a temporary TextMetrics object
//     let textMetrics = Qt.createQmlObject(
//         'import QtQuick 2.0; TextMetrics { }',
//         null,
//         "textMetrics"
//     );

//     textMetrics.font.pointSize = pointSize || 12;
//     textMetrics.font.family = fontFamily || "FiraCode";
//     textMetrics.font.bold = bold || true;
//     textMetrics.text = text;

//     let width = textMetrics.boundingRect.width;
//     textMetrics.destroy(); // Clean up

//     return width;
// }

// function calculateSubtreeWidth(node) {
//     // Calculate total width needed for this node and all its children
//     if (node.children.length === 0) {
//         return calculateNodeWidth(node)
//     }
    
//     let totalChildrenWidth = 0
//     for (let child of node.children) {
//         totalChildrenWidth += calculateSubtreeWidth(child)
//     }
    
//     // Add spacing between children
//     let childSpacing = (node.children.length - 1) * 50
//     let childrenTotalWidth = totalChildrenWidth + childSpacing
    
//     let nodeWidth = calculateNodeWidth(node)
//     return Math.max(nodeWidth, childrenTotalWidth)
// }

// // Collision Detection
// function findOptimalPosition(targetX, targetY, nodeWidth, nodeHeight, existingNodes) {
//     let testX = targetX
//     let testY = targetY
//     let collision = true
//     let attempts = 0

//     while (collision && attempts < 100) {
//         collision = false

//         for (let entry of existingNodes) {
//             let node = entry.item
//             if (!node)
//                 continue

//             let nodeLeft = node.x
//             let nodeRight = nodeLeft + node.width
//             let nodeTop = node.y
//             let nodeBottom = nodeTop + node.height

//             let testLeft = testX - nodeWidth / 2
//             let testRight = testX + nodeWidth / 2
//             let testTop = testY
//             let testBottom = testY + nodeHeight

//             // Check for overlap with 20 px padding
//             if (testLeft < nodeRight + 20 &&
//                 testRight > nodeLeft - 20 &&
//                 testTop < nodeBottom + 20 &&
//                 testBottom > nodeTop - 20) {
//                 collision = true
//                 break
//             }
//         }

//         if (collision) {
//             testX += 30
//             ++attempts

//             // If moving right doesn't work, try moving down
//             if (attempts > 20) {
//                 testX = targetX
//                 testY += 50
//             }
//         }
//     }

//     return { x: testX, y: testY }
// }
