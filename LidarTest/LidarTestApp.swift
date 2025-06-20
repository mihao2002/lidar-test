//
//  LidarTestApp.swift
//  LidarTest
//
//  Created by Hao Mi on 6/19/25.
//

import UIKit
import SwiftUI

@main
struct LidarTestApp: App {
    var body: some Scene {
        WindowGroup {
            ARViewContainer()
                .edgesIgnoringSafeArea(.all)
        }
    }
}
